#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/time.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>
// #define IMU_SENSOR
#define JOYSTICK
// #define DATALOG_MAIN
#define WEBOTS
// #define GRPC_SONNY
#define LITE_WHOLE

#include "../../SonnyRobotController/include/KalmanFilter.h"
#include "../../SonnyRobotController/include/LowPassFilter.h"
#include "broccoli/core/Time.hpp"
#include "functionIKID_S2P.h"

#include "../../../../upbody/ArmControl/include/left_arm.h"
#include "../../../../upbody/ArmControl/include/right_arm.h"

// include open source
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
// #include <qpOASES.hpp>
#include "../../MPC_Gait/include/GaitGenerator.h"
#include "../../WalkPlan/include/aeroWalkPlan.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <sys/time.h>
// #include "../../MotorList/include/MotorList.h"
// #include "../../vnIMU/include/vnIMU.h"
#ifdef JOYSTICK
    #include "../../joystick/include/joystick.h"
#endif

#ifdef ARMUDP
    #include "../../Arm_udp/include/rac.h"
#endif

#ifdef GRPC_SONNY
    #include "../../GRPC_Server/GRPC_Sonny/include/grpc_server.h"
#endif

#ifdef LITE_WHOLE
    #include "../../HeadControl/include/sonniehead.hpp"
#endif

#include "../include/webotsInterface.h"

//
#include <QCoreApplication>
#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>
#include <QStringList>
using namespace broccoli::core;

#ifndef PI
    #define PI 3.141592654
#endif // PI

void readabsoluteencoder(QString path, Eigen::VectorXd &initpos);
int main(int argc, char **argv) {
    // double dt = timeStep;
    double dt = 0.0025;
    // define data
    DataPackage *data = new DataPackage();
    // construct robot controller
    GaitGenerator gait;
    std::cout << "start json init!" << std::endl;

    // to init the task state
    QString path = "./br/sonny.json";
    QString path2 = "./WalkPlan/sources/gait_sim.json"; //"./br/mpc_settings.json";
    // std::cout<<"controller init is start!"<<std::endl;
    // controller.init(path,dt);
    gait.init(path, path2, dt, data);
    std::cout << "data init!" << std::endl;
    data->addlog("data init complete, program is starting!");
// GRPC
#ifdef GRPC_SONNY
    grpc_sonny_server grpc_server;
    grpc_server.SetDataPackage(data);
    grpc_server.start();
    std::string bodyandlegstate = "StartOn";
    grpc_server.Setbodyandlegstate(bodyandlegstate);
#endif
// imu
#ifdef IMU_SENSOR
    vnIMU imu;
    imu.initIMU();
#endif

#ifdef JOYSTICK
    Joystick_sonny joy;
    joy.init();
#endif

#ifdef LITE_WHOLE
    HeadControl sonnie_head;
#endif

#ifdef WEBOTS
#else
    // set cpu-affinity
    int cpus = 0;
    cpu_set_t mask;

    cpus = sysconf(_SC_NPROCESSORS_CONF);
    printf("cpus: %d\n", cpus);

    CPU_ZERO(&mask);          // init mask
    CPU_SET(cpus - 1, &mask); // add last cup core to cpu set

    if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
        printf("Set CPU affinity failue, ERROR:%s\n", strerror(errno));
        return -1;
    }
    usleep(1000);
    printf("set CPU affinity success\n");
    // set cpu-affinity

    // set sched-stratergy
    struct sched_param sched;

    int max_priority;

    max_priority = sched_get_priority_max(SCHED_RR);
    sched.sched_priority = max_priority;

    if (sched_setscheduler(getpid(), SCHED_RR, &sched) == -1) {
        printf("Set Scheduler Param, ERROR:%s\n", strerror(errno));
        return -1;
    }

    usleep(1000);
    printf("set scheduler success\n");
// set sched-stratergy
#endif

#ifdef SONNIE_HEAD
    int motorNum = 18;
#else
    #ifdef LITE_WHOLE
        int motorNum = 32;
        #define armNum 14
        // int armNum = 14;
        int armLeftNum = 7;
        int armRightNum = 7;
        int matain_0 = 0;
        int matain_1 = 0;
        int matain_2 = 0;
        int matain_3 = 0;
        int matain_5 = 0;
    #else
        int motorNum = 15;
    #endif
#endif

// Initialize Webots
#ifdef WEBOTS
    std::cout << "sssssssss" << endl;
    WebotsRobot Sonny_Sim;
    std::cout << "fffffffff" << endl;
    Sonny_Sim.initWebots();
    std::cout << "fffffffff" << endl;
    webotState robotStateSim;
    std::cout << "gggggggg" << endl;

    Eigen::VectorXd standPosCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd jointTorCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd jointP = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd jointD = Eigen::VectorXd::Zero(motorNum);
    double simTime = 0;
    double myInf = 100000;
    std::cout << "aaaaaaaaaaaaaaaaa" << endl;
#endif

    Time start_time;
    Time period(0, 2500000);
    Time sleep2Time;
    Time timer;
    timespec sleep2Time_spec;

#ifdef WEBOTS
#else
    functionIKID_S2P *funS2P = new functionIKID_S2P();
    Eigen::VectorXd parallelQEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd parallelQDotEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd parallelQTorEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd parallelQCmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd parallelQDotCmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd parallelQTorCmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleOrienEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleOmegaEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleTorEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleOrienRef = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleOmegaRef = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleTorDes = Eigen::VectorXd::Zero(4);
#endif

    // QString  Anglepath = "../absAngle.json";
    Eigen::VectorXd absolute_pos = Eigen::VectorXd::Zero(motorNum);

#ifdef WEBOTS
#else
    readabsoluteencoder(Anglepath, absolute_pos);

    std::vector<std::string> ip(motorNum);
    ip[0] = "10.10.10.70";
    ip[1] = "10.10.10.71";
    ip[2] = "10.10.10.72";
    ip[3] = "10.10.10.73";
    ip[4] = "10.10.10.74";
    ip[5] = "10.10.10.75";

    ip[6] = "10.10.10.50";
    ip[7] = "10.10.10.51";
    ip[8] = "10.10.10.52";
    ip[9] = "10.10.10.53";
    ip[10] = "10.10.10.54";
    ip[11] = "10.10.10.55";

    ip[12] = "10.10.10.90";
    ip[13] = "10.10.10.91";
    ip[14] = "10.10.10.92";
    //
    QString motorlistpath = "../MotorList/sources/motorlist.json";
    MotorList motorlist;
    motorlist.init(motorNum, dt, absolute_pos, ip, motorlistpath);
    motorlist.settolerance_count(3);
#endif
    //
    Eigen::VectorXd qEstDeg = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotEstDeg = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qEst = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotEst = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotEst_lowpass = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotEst_kalman = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd currentEst = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qTorEst = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDDotRef = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd currentCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qTorCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd frictioncurrentCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd frictioncurrentCmd2 = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd frictionTor = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qCmd2 = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotCmd2 = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qTorCmd2 = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qEst_p = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotEst_p = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd q_factor = Eigen::VectorXd::Ones(motorNum);
    Eigen::VectorXd qdot_factor = Eigen::VectorXd::Ones(motorNum);
    Eigen::VectorXd qDotfriciton = Eigen::VectorXd::Zero(motorNum);

#ifdef LITE_WHOLE
    Eigen::Vector3d q_head_act = Eigen::Vector3d::Zero();
    Eigen::Vector3d q_head_cmd = Eigen::Vector3d::Zero();
    Eigen::Vector3d q_head_ctr = Eigen::Vector3d::Zero();
#endif

#ifdef LITE_WHOLE
    left_arm left_arm;
    right_arm right_arm;
    int zero_over = 0;

    // planning required
    double unit_time = 0.0025;
    // mode 为 0 时
    std::array<double, armNum> new_position_0, new_velocity_0, new_accelebration_0;
    Trajectory<armNum> trajectory_0;
    double duration_0;
    // mode 为 1 时
    std::array<double, armNum> new_position_1, new_velocity_1, new_accelebration_1;
    Trajectory<armNum> trajectory_1;
    double duration_1;
    // mode 为 2 时
    std::array<double, armNum> new_position_2, new_velocity_2, new_accelebration_2;
    Trajectory<armNum> trajectory_2;
    double duration_2;
    // mode 为 3 时
    std::array<double, armNum> new_position_3, new_velocity_3, new_accelebration_3;
    Trajectory<armNum> trajectory_3;
    double duration_3;
    // mode 为 5 时
    std::array<double, armNum> new_position_5, new_velocity_5, new_accelebration_5;
    Trajectory<armNum> trajectory_5;
    double duration_5;
#endif
    // init pos
    bool flag_start = true;
    Eigen::VectorXd qEstInit = Eigen::VectorXd::Zero(motorNum);

    int simCnt = 0;
    double timeSim = 0.0;
    double timeStep = dt;
    double timeTotal = 500.0;
    int simTotalNum = timeTotal / timeStep + 10;
    // double PTPtime = 5;
    // double Ttotal = 60+PTPtime;
    // int simTotalNum = Ttotal/timeStep;
    // int simTotalNum = 20000;
    //
    int motionStartCnt = 75;     // min >= 75*0.004
    int cartMotionCycleCnt = 75; // min >= 75*0.004
//
#ifdef DATALOG_MAIN
    std::ofstream foutData;
    foutData.open("datacollection.txt", std::ios::out);
    Eigen::VectorXd dataL = Eigen::VectorXd::Zero(3 + 10 * motorNum + 50);
#endif

    /**friction compensation**/
    // -----------------------------------------------robot controller software -----------------------------------------------------------------

    // double dt = timeStep;
    // // define data

    // DataPackage * data = new DataPackage();
    // // construct robot controller
    // GaitGenerator gait;
    // std::cout<<"start json init!"<<std::endl;
    // // to init the task state
    // QString path = "./br/sonny.json";
    // QString path2 = "";
    // // std::cout<<"controller init is start!"<<std::endl;
    // // controller.init(path,dt);

    // gait.init(path,path2,dt,data);
    // // std::cout<<"task dim: "<<gait.robot_controller_._robot_data->task_card_set[gait.robot_controller_._robot_data->body_task_id]->dim<<std::endl;
    // // std::cout<<"task dim: "<<gait.robot_controller_._robot_data->task_card_set[gait.robot_controller_._robot_data->body_task_id]->X_d<<std::endl;
    // std::cout<<"data init!"<<std::endl;
    // data->addlog("data init!");

#ifdef WEBOTS
#else
    int aaaa;
    std::cin >> aaaa;
#endif
    // #ifdef GRPC_SONNY
    // grpc_sonny_server grpc_server;
    // grpc_server.SetDataPackage(data);
    // grpc_server.start();
    // #endif
    // simTotalNum = 60000;
    Time time1;
    Time time2;
    Time time3;
    Time time4;
    Time time5;
    Time time6;
#ifdef GRPC_SONNY
    bodyandlegstate = "StartComplete";
    grpc_server.Setbodyandlegstate(bodyandlegstate);
#endif

#ifdef WEBOTS
    while (Sonny_Sim.robot->step(TIME_STEP) != -1)
#else
    while (true)
#endif
    {
        start_time = timer.currentTime();
//--------------------state estimation--------------------
#ifdef WEBOTS
        simTime = Sonny_Sim.robot->getTime();
        Sonny_Sim.readData(simTime, robotStateSim);
        qEst = robotStateSim.jointPosAct.head(motorNum);
        qDotEst = robotStateSim.jointVelAct.head(motorNum);
        qTorEst = jointTorCmd.head(motorNum);
#else
        motorlist.getstate(qEst, qDotEst, qTorEst, 0);
#endif

        time1 = timer.currentTime() - start_time;

#ifdef WEBOTS
#else
        //--------------------Parrel to Serial Transform--------------------
        qEst_p = qEst;
        qDotEst_p = qDotEst;

        // std::cout<<"hehe: 1"<< std::endl;
        // p-joint to s-joint
        parallelQEst << qEst(4), qEst(5), qEst(10), qEst(11);
        parallelQDotEst << qDotEst(4), qDotEst(5), qDotEst(10), qDotEst(11);
        parallelQTorEst << qTorEst(4), qTorEst(5), qTorEst(10), qTorEst(11);
        funS2P->setEst(parallelQEst, parallelQDotEst, parallelQTorEst); // set parallel joint pos and vel
        funS2P->calcFK();                                               // calc serial joint pos
        funS2P->calcIK();                                               // calc jacobian at this pose and serial joint vel
        funS2P->getAnkleState(ankleOrienEst, ankleOmegaEst, ankleTorEst);
        qEst(4) = ankleOrienEst(0);
        qEst(5) = ankleOrienEst(1);
        qEst(10) = ankleOrienEst(2);
        qEst(11) = ankleOrienEst(3);
        qDotEst(4) = ankleOmegaEst(0);
        qDotEst(5) = ankleOmegaEst(1);
        qDotEst(10) = ankleOmegaEst(2);
        qDotEst(11) = ankleOmegaEst(3);
        qTorEst(4) = ankleTorEst(0);
        qTorEst(5) = ankleTorEst(1);
        qTorEst(10) = ankleTorEst(2);
        qTorEst(11) = ankleTorEst(3);
#endif

        //---------------------high-level control-----------------------
        //

        time2 = timer.currentTime() - start_time - time1;
        if (timeSim < 1.0) {

            for (int i = 0; i < motorNum; i++) {
                qCmd[i] = absolute_pos(i);
            }
            qDotCmd.setZero();
            q_factor = 1.0 * Eigen::VectorXd::Ones(motorNum);
            qdot_factor = 1.0 * Eigen::VectorXd::Ones(motorNum);
        } else {
            //         // set actual task variable
            data->dim = 18;
            data->dt = dt;
            // // std::cout<<"data->dt: "<<data->dt<<std::endl;
            // // right leg
            data->q_a.block(6, 0, 12, 1) = qEst.block(0, 0, 12, 1);
            data->q_dot_a.block(6, 0, 12, 1) = qDotEst.block(0, 0, 12, 1);
            data->tau_a.block(6, 0, 12, 1) = qTorEst.block(0, 0, 12, 1);
            // waist
            data->q_a_Waist = qEst.block(12, 0, 3, 1);
            // std::cout<<"q_waist_a: "<<data->q_a_Waist.transpose()<<std::endl;
            data->q_dot_a_Waist = qDotEst.block(12, 0, 3, 1);
            data->tau_a_Waist = qTorEst.block(12, 0, 3, 1);
            // data->q_waist_a = qEst.block(12,0,3,1);
            // head
            // data->q_head_a = qEst.block(15,0,3,1);

            // // std::cout<<"data->qa: "<<data->q_a.transpose()<<std::endl;
            // // std::cout<<"data->q_dot_a: "<<data->q_dot_a.transpose()<<std::endl;
            // // data->q_ddot_a.block(6,0,12,1) = RobotMotorAcc;
            // data->tau_a.block(12,0,6,1) = qTorEst;

            data->imu_sensor.block(0, 0, 18, 1).setZero();

#ifdef IMU_SENSOR
            data->imu_sensor.block(0, 0, 9, 1) = vnIMU::imuData;
#endif

#ifdef WEBOTS
            data->imu_sensor.block(0, 0, 9, 1) = robotStateSim.imu9DAct;
#endif

#ifdef JOYSTICK
            if (gait.fsmstatename == "S2W" || gait.fsmstatename == "Z2S") {

            } else {
                gait.setevent(joy.get_state_change());
                gait.set_current_fsm_command(joy.get_current_state_command());
            }
            if (gait.fsmstatename == "Walk") {
                gait.setvelocity(joy.get_walk_x_direction_speed(),
                                 joy.get_walk_y_direction_speed(),
                                 joy.get_walk_yaw_direction_speed());
                gait.setvelocity_offset(joy.get_walk_x_direction_speed_offset(),
                                        joy.get_walk_y_direction_speed_offset());
            } else if (gait.fsmstatename == "Stand") {
                gait.setxyz(joy.get_stand_x_direction_position(),
                            joy.get_stand_y_direction_posiiton(),
                            joy.get_stand_z_direction_posiiton());
                gait.setrollpitch(joy.get_stand_roll_direction_position(),
                                  joy.get_stand_pitch_direction_posiiton(),
                                  joy.get_stand_yaw_direction_posiiton());
            } else {
            }
#endif

#ifdef GRPC_SONNY
            if (gait.fsmstatename == "S2W" || gait.fsmstatename == "Z2S") {
                if (grpc_server.GetStateCommand() == "Stop") {
                    gait.setevent(grpc_server.GetStateCommand());
                    gait.set_current_fsm_command(grpc_server.GetStateCommand());
                }
            } else {
                gait.setevent(grpc_server.GetStateCommand());
                gait.set_current_fsm_command(grpc_server.GetStateCommand());
                // std::cout<<"event: "<<gait.event<<std::endl;
            }
            if (gait.fsmstatename == "Walk") {
                gait.setvelocity(grpc_server.get_walk_x_direction_speed(),
                                 grpc_server.get_walk_y_direction_speed(),
                                 grpc_server.get_walk_yaw_direction_speed());
                // std::cout<<"vx: "<<grpc_server.v_cmd(0)<<std::endl;
                // std::cout<<"vyaw: "<<grpc_server.v_cmd(2)<<std::endl;
            } else if (gait.fsmstatename == "Stand") {
                gait.setxyz(grpc_server.get_stand_x_direction_position(),
                            grpc_server.get_stand_y_direction_position(),
                            grpc_server.get_stand_z_direction_position());
                gait.setrollpitch(grpc_server.get_stand_roll_direction_position(),
                                  grpc_server.get_stand_pitch_direction_position(),
                                  grpc_server.get_stand_yaw_direction_position());
    #ifdef LITE_WHOLE
                    q_head_cmd(0) = grpc_server.get_stand_head_yaw_direction_position();
                    q_head_cmd(1) = grpc_server.get_stand_head_roll_direction_position();
                    q_head_cmd(2) = grpc_server.get_stand_head_pitch_direction_position();
    #endif
                } else {
                }
#endif
            gait.gait_run(data);
#ifdef  LITE_WHOLE
            q_head_act = qEst.block(15, 0, 3, 1);
            q_head_ctr = sonnie_head.JoyStickControl(q_head_cmd, q_head_act);
            qCmd.block(15, 0, 3, 1) = q_head_ctr;
#endif

            qCmd.block(0, 0, 12, 1) = data->q_c.block(6, 0, 12, 1);
            qCmd.block(12, 0, 3, 1) = data->q_waist_c;

            qDotCmd.block(0, 0, 12, 1) = data->q_dot_c.block(6, 0, 12, 1);
            qDotCmd.block(12, 0, 3, 1) = data->q_dot_c_Waist;

            qTorCmd.block(0, 0, 12, 1) = data->tau_c.block(6, 0, 12, 1);
            qTorCmd.block(12, 0, 3, 1) = data->tau_c_Waist;

            q_factor.block(0, 0, 12, 1) = data->q_factor;
            q_factor.block(12, 0, 3, 1) = data->q_factor_Waist;

            qdot_factor.block(0, 0, 12, 1) = data->q_dot_factor;
            qdot_factor.block(12, 0, 3, 1) = data->q_dot_factor_Waist;

#ifdef WEBOTS
#else
            //--------------------Serial to Parrel Transform--------------------
            ankleOrienRef(0) = qCmd(4);
            ankleOrienRef(1) = qCmd(5);
            ankleOrienRef(2) = qCmd(10);
            ankleOrienRef(3) = qCmd(11);

            ankleOmegaRef(0) = qDotCmd(4);
            ankleOmegaRef(1) = qDotCmd(5);
            ankleOmegaRef(2) = qDotCmd(10);
            ankleOmegaRef(3) = qDotCmd(11);
            time3 = timer.currentTime() - start_time - time2 - time1;
            // // s2p
            ankleTorDes << qTorCmd(4), qTorCmd(5), qTorCmd(10), qTorCmd(11);
            funS2P->setDes(ankleOrienRef, ankleOmegaRef);
            funS2P->calcJointPosRef();
            funS2P->setDesTorque(ankleTorDes);
            funS2P->calcJointTorDes();
            funS2P->getDes(parallelQCmd, parallelQDotCmd, parallelQTorCmd);
            qCmd(4) = parallelQCmd(0);
            qCmd(5) = parallelQCmd(1);
            qCmd(10) = parallelQCmd(2);
            qCmd(11) = parallelQCmd(3);
            qDotCmd(4) = parallelQDotCmd(0);
            qDotCmd(5) = parallelQDotCmd(1);
            qDotCmd(10) = parallelQDotCmd(2);
            qDotCmd(11) = parallelQDotCmd(3);
            qTorCmd(4) = parallelQTorCmd(0);
            qTorCmd(5) = parallelQTorCmd(1);
            qTorCmd(10) = parallelQTorCmd(2);
            qTorCmd(11) = parallelQTorCmd(3);
#endif
            //----------------------------------//
            time4 = timer.currentTime() - start_time - time3 - time2 - time1;
        }
#ifdef LITE_WHOLE

        // current position  dealing data
        cur_pos_left_1 = qEst[18];
        cur_pos_left_2 = qEst[19];
        cur_pos_left_3 = qEst[20];
        cur_pos_left_4 = qEst[21];
        cur_pos_left_5 = qEst[22];
        cur_pos_left_6 = qEst[23];
        cur_pos_left_7 = qEst[24];

        cur_pos_right_1 = qEst[25];
        cur_pos_right_2 = qEst[26];
        cur_pos_right_3 = qEst[27];
        cur_pos_right_4 = qEst[28];
        cur_pos_right_5 = qEst[29];
        cur_pos_right_6 = qEst[30];
        cur_pos_right_7 = qEst[31];f
        cur_vel_left_1 = qDotEst[18];
        cur_vel_left_2 = qDotEst[19];
        cur_vel_left_3 = qDotEst[20];
        cur_vel_left_4 = qDotEst[21];
        cur_vel_left_5 = qDotEst[22];
        cur_vel_left_6 = qDotEst[23];
        cur_vel_left_7 = qDotEst[24];

        cur_vel_right_1 = qDotEst[25];
        cur_vel_right_2 = qDotEst[26];
        cur_vel_right_3 = qDotEst[27];
        cur_vel_right_4 = qDotEst[28];
        cur_vel_right_5 = qDotEst[29];
        cur_vel_right_6 = qDotEst[30];
        cur_vel_right_7 = qDotEst[31];



        // logical judgment

        if (zero_over == 0) {
            qCmd.block(18, 0, 7, 1) = left_arm.GotoZero(qEst.block(18, 0, 7, 1), zero_over);
            qCmd.block(25, 0, 7, 1) = right_arm.GotoZero(qEst.block(25, 0, 7, 1), zero_over);
        } else if (zero_over == 1) {

            if (arm_mode == 0) {
                if (first_enter_0) {
                    // 计算轨迹
                    Ruckig<armNum> otg;
                    InputParameter<armNum> input;
                    OutputParameter<armNum> output;
                    input.current_position = {cur_pos_left_1, cur_pos_left_2, cur_pos_left_3, cur_pos_left_4, cur_pos_left_5, cur_pos_left_6, cur_pos_left_7, 
                                                cur_pos_right_1, cur_pos_right_2, cur_pos_right_3, cur_pos_right_4, cur_pos_right_5, cur_pos_right_6, cur_pos_right_7};

                    input.current_velocity = {cur_vel_left_1, cur_vel_left_2, cur_vel_left_3, cur_vel_left_4, cur_vel_left_5, cur_vel_left_6, cur_vel_left_7,
                                                cur_vel_right_1, cur_vel_right_2, cur_vel_right_3, cur_vel_right_4, cur_vel_right_5, cur_vel_right_6, cur_vel_right_7};

                    input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.target_position = {left_arm.file_vec_left[401][0], left_arm.file_vec_left[401][1], left_arm.file_vec_left[401][2], 
                                                left_arm.file_vec_left[401][3], left_arm.file_vec_left[401][4], left_arm.file_vec_left[401][5], left_arm.file_vec_left[401][6],
                                             right_arm.file_vec_right[401][0], right_arm.file_vec_right[401][1], right_arm.file_vec_right[401][2], 
                                             right_arm.file_vec_right[401][3], right_arm.file_vec_right[401][4], right_arm.file_vec_right[401][5], right_arm.file_vec_right[401][6]};

                    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.max_velocity = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                            1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.max_acceleration = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                            1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.max_jerk = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                            1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.min_velocity = {-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 
                                            -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57};

                    input.min_acceleration = {-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 
                                            -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57};

                    Result result_0 = otg.calculate(input, trajectory_0);
                    trajectory_0.at_time(seq_num_0 * unit_time, new_position_0, new_velocity_0, new_accelebration_0);
                    for (int i = 0; i < armNum; i++) {
                            qCmd[18 + i] = new_position_0[i];
                        }
                    
                    duration_0 = trajectory_0.get_duration();
                    seq_num_0++;
                    first_enter_0 = false;
                    upper_action = true;
                }
                if (!first_enter_0 and seq_num_0 * unit_time < duration_0) {
                    trajectory_0.at_time(seq_num_0 * unit_time, new_position_0, new_velocity_0, new_accelebration_0);
                    for (int i = 0; i < armNum; i++) {
                        qCmd[18 + i] = new_position_0[i];
                        }
                    
                    seq_num_0++;
                    upper_action = true;
                }
                if (!first_enter_0 and seq_num_0 * unit_time >= duration_0) {
                    for (int i = 0; i < armNum; i++) 
                    {
                        qCmd[18 + i] = new_position_0[i];
                    }
                    arm_mode = 100;
                    upper_action = true;
                }
                
            } else if (arm_mode == 1) {
                if (first_enter_1) {
                    // 计算轨迹
                    Ruckig<armNum> otg;
                    InputParameter<armNum> input;
                    OutputParameter<armNum> output;
                    input.current_position = {cur_pos_left_1, cur_pos_left_2, cur_pos_left_3, cur_pos_left_4, cur_pos_left_5, cur_pos_left_6, cur_pos_left_7, 
                                                cur_pos_right_1, cur_pos_right_2, cur_pos_right_3, cur_pos_right_4, cur_pos_right_5, cur_pos_right_6, cur_pos_right_7};
                    input.current_velocity = {cur_vel_left_1, cur_vel_left_2, cur_vel_left_3, cur_vel_left_4, cur_vel_left_5, cur_vel_left_6, cur_vel_left_7,
                                                cur_vel_right_1, cur_vel_right_2, cur_vel_right_3, cur_vel_right_4, cur_vel_right_5, cur_vel_right_6, cur_vel_right_7};
                    input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.target_position = {left_arm.file_vec_left_1[0][0], left_arm.file_vec_left_1[0][1], left_arm.file_vec_left_1[0][2], left_arm.file_vec_left_1[0][3],
                                                left_arm.file_vec_left_1[0][4], left_arm.file_vec_left_1[0][5], left_arm.file_vec_left_1[0][6],
                                             right_arm.file_vec_right_1[0][0], right_arm.file_vec_right_1[0][1], right_arm.file_vec_right_1[0][2], 
                                             right_arm.file_vec_right_1[0][3], right_arm.file_vec_right_1[0][4], right_arm.file_vec_right_1[0][5], right_arm.file_vec_right_1[0][6]};
                    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.max_velocity = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                            1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.max_acceleration = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                                1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.max_jerk = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                        1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.min_velocity = {-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 
                                            -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57};

                    input.min_acceleration = {-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 
                                                -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57};

                    Result result_1 = otg.calculate(input, trajectory_1);

                    trajectory_1.at_time(seq_num_1 * unit_time, new_position_1, new_velocity_1, new_accelebration_1);
                    for (int i = 0; i < armNum; i++) {
                            qCmd[18 + i] = new_position_1[i];
                    }

                    duration_1 = trajectory_1.get_duration();
                    seq_num_1++;
                    first_enter_1 = false;
                    upper_action = true;
                }
                if (!first_enter_1 and seq_num_1 * unit_time < duration_1) {
                    trajectory_1.at_time(seq_num_1 * unit_time, new_position_1, new_velocity_1, new_accelebration_1);
                    for (int i = 0; i < armNum; i++) {
                        qCmd[18 + i] = new_position_1[i];
                    }
                    seq_num_1++;
                    upper_action = true;
                }
                // 开始读文件
                if ((!first_enter_1 and seq_num_1 * unit_time >= duration_1) or (duration_1 <= 0.0)) {
                    if (row_execute_1 >= left_arm.file_vec_left_1.size() - 1) {
                        qCmd.block(18, 0, 7, 1) = left_arm.Dance(row_execute_1, left_arm.file_vec_left_1);
                        qCmd.block(25, 0, 7, 1) = right_arm.Dance(row_execute_1, right_arm.file_vec_right_1);
                        matain_1 += 1;
                        if (matain_1 == 200)
                        {
                            arm_mode = 0;
                            first_enter_0 = true;
                            upper_action = true;
                            matain_1 = 0;
                        }
                        
                    }
                    if (row_execute_1 < left_arm.file_vec_left_1.size() - 1) {
                        qCmd.block(18, 0, 7, 1) = left_arm.Dance(row_execute_1, left_arm.file_vec_left_1);
                        qCmd.block(25, 0, 7, 1) = right_arm.Dance(row_execute_1, right_arm.file_vec_right_1);
                        row_execute_1++;
                        upper_action = true;
                    }
                }
            } else if (arm_mode == 2) {
                if (first_enter_2) {
                    // 计算轨迹
                    Ruckig<armNum> otg;
                    InputParameter<armNum> input;
                    OutputParameter<armNum> output;
                    input.current_position = {cur_pos_left_1, cur_pos_left_2, cur_pos_left_3, cur_pos_left_4, cur_pos_left_5, cur_pos_left_6, cur_pos_left_7,
                                                cur_pos_right_1, cur_pos_right_2, cur_pos_right_3, cur_pos_right_4, cur_pos_right_5, cur_pos_right_6, cur_pos_right_7};

                    input.current_velocity = {cur_vel_left_1, cur_vel_left_2, cur_vel_left_3, cur_vel_left_4, cur_vel_left_5, cur_vel_left_6, cur_vel_left_7,
                                                cur_vel_right_1, cur_vel_right_2, cur_vel_right_3, cur_vel_right_4, cur_vel_right_5, cur_vel_right_6, cur_vel_right_7};

                    input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.target_position = {left_arm.file_vec_left_2[0][0], left_arm.file_vec_left_2[0][1], left_arm.file_vec_left_2[0][2], left_arm.file_vec_left_2[0][3],
                                                left_arm.file_vec_left_2[0][4], left_arm.file_vec_left_2[0][5], left_arm.file_vec_left_2[0][6],
                                             right_arm.file_vec_right_2[0][0], right_arm.file_vec_right_2[0][1], right_arm.file_vec_right_2[0][2], right_arm.file_vec_right_2[0][3],
                                             right_arm.file_vec_right_2[0][4], right_arm.file_vec_right_2[0][5], right_arm.file_vec_right_2[0][6]};

                    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.max_velocity = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                            1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.max_acceleration = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                               1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.max_jerk = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                        1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.min_velocity = {-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57,
                                             -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57};

                    input.min_acceleration = {-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 
                                                -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57};

                    Result result_2 = otg.calculate(input, trajectory_2);
                    trajectory_2.at_time(seq_num_2 * unit_time, new_position_2, new_velocity_2, new_accelebration_2);
                    for (int i = 0; i < armNum; i++) {
                        qCmd[18 + i] = new_position_2[i];
                    }
                    duration_2 = trajectory_2.get_duration();
                    seq_num_2++;
                    first_enter_2 = false;
                    upper_action = true;
                }
                if (!first_enter_2 and seq_num_2 * unit_time < duration_2) {
                    trajectory_2.at_time(seq_num_2 * unit_time, new_position_2, new_velocity_2, new_accelebration_2);
                    for (int i = 0; i < armNum; i++) {
                        qCmd[18 + i] = new_position_2[i];
                    }
                    seq_num_2++;
                    upper_action = true;
                }
                // 开始读文件
                if (!first_enter_2 and seq_num_2 * unit_time >= duration_2) {
                    if (row_execute_2 >= left_arm.file_vec_left_2.size() - 1) {
                        qCmd.block(18, 0, 7, 1) = left_arm.Dance(row_execute_2, left_arm.file_vec_left_2);
                        qCmd.block(25, 0, 7, 1) = right_arm.Dance(row_execute_2, right_arm.file_vec_right_2);
                        matain_2 += 1;
                        if (matain_2 == 200)
                        {
                            arm_mode = 0;
                            first_enter_0 = true;
                            upper_action = true;
                            matain_2 = 0;
                        }
                        
                    }
                    if (row_execute_2 < left_arm.file_vec_left_2.size() - 1) {
                        qCmd.block(18, 0, 7, 1) = left_arm.Dance(row_execute_2, left_arm.file_vec_left_2);
                        qCmd.block(25, 0, 7, 1) = right_arm.Dance(row_execute_2, right_arm.file_vec_right_2);
                        row_execute_2++;
                        upper_action = true;
                    }
                }
            }

            else if (arm_mode == 3) {
                if (first_enter_3) {
                    // 计算轨迹
                    Ruckig<armNum> otg;
                    InputParameter<armNum> input;
                    OutputParameter<armNum> output;
                    input.current_position = {cur_pos_left_1, cur_pos_left_2, cur_pos_left_3, cur_pos_left_4, cur_pos_left_5, cur_pos_left_6, cur_pos_left_7,
                                                    cur_pos_right_1, cur_pos_right_2, cur_pos_right_3, cur_pos_right_4, cur_pos_right_5, cur_pos_right_6, cur_pos_right_7};

                    input.current_velocity = {cur_vel_left_1, cur_vel_left_2, cur_vel_left_3, cur_vel_left_4, cur_vel_left_5, cur_vel_left_6, cur_vel_left_7,
                                                    cur_vel_right_1, cur_vel_right_2, cur_vel_right_3, cur_vel_right_4, cur_vel_right_5, cur_vel_right_6, cur_vel_right_7};

                    input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.target_position = {left_arm.file_vec_left_3[0][0], left_arm.file_vec_left_3[0][1], left_arm.file_vec_left_3[0][2], left_arm.file_vec_left_3[0][3],
                                                left_arm.file_vec_left_3[0][4], left_arm.file_vec_left_3[0][5], left_arm.file_vec_left_3[0][6],
                                             right_arm.file_vec_right_3[0][0], right_arm.file_vec_right_3[0][1], right_arm.file_vec_right_3[0][2], right_arm.file_vec_right_3[0][3],
                                                right_arm.file_vec_right_3[0][4], right_arm.file_vec_right_3[0][5], right_arm.file_vec_right_3[0][6]};
                    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.max_velocity = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                            1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.max_acceleration = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                                1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.max_jerk = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                            1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.min_velocity = {-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 
                                            -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57};

                    input.min_acceleration = {-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 
                                            -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57};

                    Result result_3 = otg.calculate(input, trajectory_3);
                    trajectory_3.at_time(seq_num_3 * unit_time, new_position_3, new_velocity_3, new_accelebration_3);
                    for (int i = 0; i < armNum; i++) {
                            qCmd[18 + i] = new_position_3[i];
                    }
                    duration_3 = trajectory_3.get_duration();
                    seq_num_3++;
                    first_enter_3 = false;
                    upper_action = true;
                }
                if (!first_enter_3 and seq_num_3 * unit_time < duration_3) {
                    trajectory_3.at_time(seq_num_3 * unit_time, new_position_3, new_velocity_3, new_accelebration_3);
                    for (int i = 0; i < armNum; i++) {
                        qCmd[18 + i] = new_position_3[i];
                    }
                    seq_num_3++;
                    upper_action = true;
                }
                // 开始读文件
                if (!first_enter_3 and seq_num_3 * unit_time >= duration_3) {
                    if (row_execute_3 >= left_arm.file_vec_left_3.size() - 1) {
                        qCmd.block(18, 0, 7, 1) = left_arm.Dance(row_execute_3, left_arm.file_vec_left_3);
                        qCmd.block(25, 0, 7, 1) = right_arm.Dance(row_execute_3, right_arm.file_vec_right_3);
                        matain_3 += 1;
                        if (matain_3 == 200)
                        {
                            arm_mode = 0;
                            first_enter_0 = true;
                            upper_action = true;
                            matain_3 = 0;
                        }
                    }
                    if (row_execute_3 < left_arm.file_vec_left_3.size() - 1) {
                        qCmd.block(18, 0, 7, 1) = left_arm.Dance(row_execute_3, left_arm.file_vec_left_3);
                        qCmd.block(25, 0, 7, 1) = right_arm.Dance(row_execute_3, right_arm.file_vec_right_3);
                        row_execute_3++;
                        upper_action = true;
                    }
                }
            }

            else if (arm_mode == 5) {
                if (first_enter_5) {
                    // 计算轨迹
                    Ruckig<armNum> otg;
                    InputParameter<armNum> input;
                    OutputParameter<armNum> output;
                    input.current_position = {cur_pos_left_1, cur_pos_left_2, cur_pos_left_3, cur_pos_left_4, cur_pos_left_5, cur_pos_left_6, cur_pos_left_7,
                                                cur_pos_right_1, cur_pos_right_2, cur_pos_right_3, cur_pos_right_4, cur_pos_right_5, cur_pos_right_6, cur_pos_right_7};

                    input.current_velocity = {cur_vel_left_1, cur_vel_left_2, cur_vel_left_3, cur_vel_left_4, cur_vel_left_5, cur_vel_left_6, cur_vel_left_7, 
                                                cur_vel_right_1, cur_vel_right_2, cur_vel_right_3, cur_vel_right_4, cur_vel_right_5, cur_vel_right_6, cur_vel_right_7};

                    input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.target_position = {left_arm.file_vec_left_5[0][0], left_arm.file_vec_left_5[0][1], left_arm.file_vec_left_5[0][2], left_arm.file_vec_left_5[0][3],
                                                    left_arm.file_vec_left_5[0][4], left_arm.file_vec_left_5[0][5], left_arm.file_vec_left_5[0][6],
                                             right_arm.file_vec_right_5[0][0], right_arm.file_vec_right_5[0][1], right_arm.file_vec_right_5[0][2], right_arm.file_vec_right_5[0][3],
                                                        right_arm.file_vec_right_5[0][4], right_arm.file_vec_right_5[0][5], right_arm.file_vec_right_5[0][6]};

                    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    input.max_velocity = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                            1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.max_acceleration = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                                1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.max_jerk = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 
                                        1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57};

                    input.min_velocity = {-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 
                                            -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57};

                    input.min_acceleration = {-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 
                                                -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57};

                    Result result_5 = otg.calculate(input, trajectory_5);
                    trajectory_5.at_time(seq_num_5 * unit_time, new_position_5, new_velocity_5, new_accelebration_5);
                    for (int i = 0; i < armNum; i++) {
                            qCmd[18 + i] = new_position_5[i];
                    }
                    duration_5 = trajectory_5.get_duration();
                    seq_num_5++;
                    first_enter_5 = false;
                    upper_action = true;
                }
                if (!first_enter_5 and seq_num_5 * unit_time < duration_5) {
                    trajectory_5.at_time(seq_num_5 * unit_time, new_position_5, new_velocity_5, new_accelebration_5);
                    for (int i = 0; i < 8; i++) {
                        qCmd[18 + i] = new_position_5[i];
                    }
                    seq_num_5++;
                    upper_action = true;
                }
                // 开始读文件
                if (!first_enter_5 and seq_num_5 * unit_time >= duration_5) {
                    if (row_execute_5 >= left_arm.file_vec_left_5.size() - 1) {
                        qCmd.block(18, 0, 7, 1) = left_arm.Dance(row_execute_5, left_arm.file_vec_left_5);
                        qCmd.block(25, 0, 7, 1) = right_arm.Dance(row_execute_5, right_arm.file_vec_right_5);
                        matain_5 += 1;
                        if (matain_5 == 200)
                        {
                            arm_mode = 0;
                            first_enter_0 = true;
                            upper_action = true;
                            matain_5 = 0;
                        }
                    }
                    if (row_execute_5 < left_arm.file_vec_left_5.size() - 1) {
                        qCmd.block(18, 0, 7, 1) = left_arm.Dance(row_execute_5, left_arm.file_vec_left_5);
                        qCmd.block(25, 0, 7, 1) = right_arm.Dance(row_execute_5, right_arm.file_vec_right_5);
                        row_execute_5++;
                        upper_action = true;
                    }
                }
            }

            else if(arm_mode == 100) {
                qCmd.block(18, 0, 7, 1) = left_arm.Swing(qEst.block(18, 0, 7, 1), gait.getleftamp(), gait.getleftphase());
                qCmd.block(25, 0, 7, 1) = right_arm.Swing(qEst.block(25, 0, 7, 1), gait.getrightamp(), gait.getrightphase());
                upper_action = false;
                all_init = true;
            }
        }
        data->q_hand_a = qCmd.block(18, 0, armNum, 1);
#endif

        // //
        // --------------------friction compensation-------------------
        // qCmd.setZero();
        // qDotCmd.setZero();
        // qTorCmd.setZero();

        // std::cout<<"hehe: 3"<< std::endl;
        // qTorCmd2.setZero();
        // motorlist.setcommand(qCmd2,qDotCmd2,qTorCmd2,2,0);
        qDotfriciton = qDotCmd;
        qDotfriciton(4) = 0.0;
        qDotfriciton(5) = 0.0;
        qDotfriciton(10) = 0.0;
        qDotfriciton(11) = 0.0;
        // qDotfriciton = qDotEst_p;

#ifdef WEBOTS
        // jointP = 300.0*Eigen::VectorXd::Ones(23);
        // jointD = 0.1*Eigen::VectorXd::Ones(23);
        // jointP.head(12) = 40.0*Eigen::VectorXd::Ones(12);
        // jointD.head(12) = 0.01*Eigen::VectorXd::Ones(12);
        // jointP << 1314.0, 1314.0, 502.2, 297.6, 188.0, 52.64, 1314.0, 1314.0, 502.2, 297.6, 188.0, 52.64, 1314.0, 1314.0, 1314.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
        // jointD << 26.28, 26.28, 16.74, 14.88, 3.76, 1.053, 26.28, 26.28, 16.74, 14.88, 3.76, 1.053, 26.28, 26.28, 26.28, 1.88, 1.88, 1.88, 1.88, 1.88, 1.88, 1.88, 1.88;

        // jointP << 1000.0, 1000.0, 502.2, 297.6, 4.0, 1.0, 
        //         1000.0, 1000.0, 502.2, 297.6, 4.0, 1.0, 
        //         1000.0, 1000.0, 1000.0, 
        //         100.0, 100.0, 100.0, 
        //         100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 
        //         100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
        // jointD << 10.0, 10.0, 10.0, 10.0, 0.1, 0.025, 
        //         10.0, 10.0, 10.0, 10.0, 0.1, 0.025, 
        //         10.0, 10.0, 10.0, 
        //         10, 10, 10, 
        //         10, 10, 10, 10, 10, 10, 10, 
        //         10, 10, 10, 10, 10, 10, 10;
        jointP << 1000.0, 1000.0, 502.2, 297.6, 10.0, 1.0, 
                1000.0, 1000.0, 502.2, 297.6, 10.0, 1.0, 
                1000.0, 1000.0, 1000.0, 
                100.0, 100.0, 100.0, 
                100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 
                100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
        jointD << 10.0, 10.0, 10.0, 10.0, 0.1, 0.025, 
                10.0, 10.0, 10.0, 10.0, 0.1, 0.025, 
                10.0, 10.0, 10.0, 
                10, 10, 10, 
                10, 10, 10, 10, 10, 10, 10, 
                10, 10, 10, 10, 10, 10, 10;

#ifdef WEBOTS
        for (int i = 0; i < motorNum; i++) {
            qCmd2(i) = (1.0 - q_factor(i)) * qEst(i) + q_factor(i) * qCmd(i);
            qDotCmd2(i) = (1.0 - qdot_factor(i)) * qDotEst(i) + qdot_factor(i) * qDotCmd(i);
        }
        qTorCmd2 = qTorCmd;
#else
        for (int i = 0; i < motorNum; i++) {
            qCmd2(i) = (1.0 - q_factor(i)) * qEst_p(i) + q_factor(i) * qCmd(i);
            qDotCmd2(i) = (1.0 - qdot_factor(i)) * qDotEst_p(i) + qdot_factor(i) * qDotCmd(i);
        }
        qTorCmd2 = qTorCmd;
#endif

        for (int i = 0; i < motorNum; i++) {
            // if(i < 15)
            // {
            jointTorCmd(i) = jointP(i) * (qCmd2(i) - qEst(i)) + jointD(i) * (qDotCmd2(i) - qDotEst(i)) + qTorCmd2(i);
            standPosCmd(i) = qCmd2(i); // myInf;
            // }
        }
        // for(int i=motorNum; i < motorNum+8; i++){
        //     jointTorCmd(i) = myInf;
        //     standPosCmd(i) = 0;
        // }
        if (timeSim < 1.0 || gait.fsmstatename == "Start" || gait.fsmstatename == "Zero") {
            Sonny_Sim.setMotorPos(standPosCmd);
        } else {
            Sonny_Sim.setMotorTau(jointTorCmd, standPosCmd);
        }
        // Sonny_Sim.setMotorPos(standPosCmd);
        // Sonny_Sim.setMotorTau(jointTorCmd);
#else
        motorlist.setcommand(qCmd2, qDotCmd2, qTorCmd2, 1, 0, qDotfriciton);
#endif
        // std::cout<<"hehe: 4"<< std::endl;
        simCnt += 1;
        timeSim = simCnt * timeStep;
        time5 = timer.currentTime() - start_time - time4 - time3 - time2 - time1;
//
#ifdef DATALOG_MAIN
        dataL[0] = timeSim;
        dataL.block(1, 0, motorNum, 1) = qCmd;
        dataL.block(1 + motorNum, 0, motorNum, 1) = qDotCmd;
        dataL.block(1 + 2 * motorNum, 0, motorNum, 1) = qTorCmd;
        dataL.block(1 + 3 * motorNum, 0, motorNum, 1) = qEst;
        dataL.block(1 + 4 * motorNum, 0, motorNum, 1) = qDotEst;
        dataL.block(1 + 5 * motorNum, 0, motorNum, 1) = qTorEst;
        dataL.block(1 + 6 * motorNum, 0, 9, 1) = robotStateSim.imu9DAct;
// dataL.block(1+6*motorNum,0,motorNum,1) = frictionTor;//frictioncurrentCmd;
// dataL.block(1+7*motorNum,0,motorNum,1) = frictioncurrentCmd2;
// dataL.block(1+8*motorNum,0,motorNum,1) = qDotEst_lowpass;
// dataL.block(1+9*motorNum,0,motorNum,1) = qDotEst_kalman;
// dataL[1+10*motorNum] = start_time.m_seconds;
// dataL[1+10*motorNum+1] = start_time.m_nanoSeconds;
// dataL.block(1+10*motorNum+2,0,4,1) = parallelQEst;
// dataL.block(1+10*motorNum+6,0,4,1) = parallelQDotEst;
// dataL.block(1+10*motorNum+10,0,4,1) = ankleOrienRef;
// dataL.block(1+10*motorNum+14,0,4,1) = ankleOmegaRef;
// dataL.block(1+10*motorNum+18,0,4,1) = ankleOrienEst;
// dataL.block(1+10*motorNum+22,0,4,1) = ankleOmegaEst;

// dataL.block(1+10*motorNum+26,0,4,1) = parallelQTorEst;
// dataL.block(1+10*motorNum+30,0,9,1) = vnIMU::imuData;
// dataL(1+10*motorNum+39) = time1.m_nanoSeconds;
// dataL(1+10*motorNum+40) = time2.m_nanoSeconds;
// dataL(1+10*motorNum+41) = time3.m_nanoSeconds;
// dataL(1+10*motorNum+42) = time4.m_nanoSeconds;
// dataL(1+10*motorNum+43) = time5.m_nanoSeconds;
// dataL(1+10*motorNum+44) = time6.m_nanoSeconds;
#ifdef ARMUDP
        dataL(1 + 10 * motorNum + 45) = gait.getleftphase();
        dataL(1 + 10 * motorNum + 46) = gait.getrightphase();
#endif
        dataLog(dataL, foutData); 1);
            // qCmd.block(21, 0, 1, 1) = -qCmd.block(21, 0, 1, 1);
        sleep2Time = start_time + period;
        sleep2Time_spec = sleep2Time.toTimeSpec();
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(sleep2Time_spec), NULL);
#endif

        if (timeSim > timeTotal) {
            break;
        }
    }

#ifdef WEBOTS
    Sonny_Sim.deleteRobot();
#else
    motorlist.disable();
#endif
#ifdef IMU_SENSOR
// imu.closeIMU();
#endif
#ifdef DATALOG_MAIN
    foutData.close();
#endif
    return 0;
}

// plan tool
//  right leg
void readabsoluteencoder(QString path, Eigen::VectorXd &initpos) {
    // read the json file
    QFile loadFile(path);
    if (!loadFile.open(QIODevice::ReadOnly)) {
        qDebug() << "could't open projects json";
        return;
    }

    QByteArray allData = loadFile.readAll();
    loadFile.close();

    QJsonParseError jsonerror;
    QJsonDocument doc(QJsonDocument::fromJson(allData, &jsonerror));

    if (jsonerror.error != QJsonParseError::NoError) {
        qDebug() << "json error!" << jsonerror.errorString();
        return;
    }

    if (!doc.isNull() && jsonerror.error == QJsonParseError::NoError) {
        if (doc.isObject()) {
            QJsonObject object = doc.object();
            QJsonObject::iterator it = object.begin();
            // read the dimesion of the variables
            while (it != object.end()) {
                if (it.key() == "192.168.31.70") {
                    QJsonObject it_object = it.value().toObject();
                    QJsonObject::iterator iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(0) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.31.71") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(1) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.31.72") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(2) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.31.73") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(3) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.31.74") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(4) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.31.75") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(5) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.31.50") {
                    QJsonObject it_object = it.value().toObject();
                    QJsonObject::iterator iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(6) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.31.51") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(7) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.31.52") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(8) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.31.53") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(9) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.31.54") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(10) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.31.55") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(11) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                it++;
            }
        }
    }
}

// void readabsoluteencoder(QString path, Eigen::VectorXd &initpos)
// {
//         // read the json file
//     QFile loadFile(path);
//     if(!loadFile.open(QIODevice::ReadOnly))
//     {
//         qDebug() << "could't open projects json";
//         return;
//     }

//     QByteArray allData = loadFile.readAll();
//     loadFile.close();

//     QJsonParseError jsonerror;
//     QJsonDocument doc(QJsonDocument::fromJson(allData, &jsonerror));

//     if(jsonerror.error != QJsonParseError::NoError)
//     {
//         qDebug() << "json error!" << jsonerror.errorString();
//         return;
//     }

//     if (!doc.isNull() && jsonerror.error == QJsonParseError::NoError)
//     {
//         if(doc.isObject())
//         {
//             QJsonObject object = doc.object();
//             QJsonObject::iterator it = object.begin();
//             // read the dimesion of the variables
//             while(it != object.end())
//             {
//                 if(it.key()=="2C5EDA25754C"){
//                     QJsonObject it_object = it.value().toObject();
//                     QJsonObject::iterator iter = it_object.begin();
//                     while(iter!=it_object.end())
//                     {
//                         if(iter.key() == "radian"){
//                             initpos(0) = iter.value().toDouble();
//                         }
//                         iter++;
//                     }
//                 }
//                 if(it.key() == "D05EDA25754C"){
//                     QJsonObject it_object = it.value().toObject();
//                     auto iter = it_object.begin();
//                     while(iter!=it_object.end())
//                     {
//                         if(iter.key() == "radian"){
//                             initpos(1) = iter.value().toDouble();
//                         }
//                         iter++;
//                     }

//                 }
//                 if(it.key() == "0450DA25754C"){
//                     QJsonObject it_object = it.value().toObject();
//                     auto iter = it_object.begin();
//                     while(iter!=it_object.end())
//                     {
//                         if(iter.key() == "radian"){
//                             initpos(2) = iter.value().toDouble();
//                         }
//                         iter++;
//                     }

//                 }
//                 if(it.key() == "FC4FDA25754C"){
//                     QJsonObject it_object = it.value().toObject();
//                     auto iter = it_object.begin();
//                     while(iter!=it_object.end())
//                     {
//                         if(iter.key() == "radian"){
//                             initpos(3) = iter.value().toDouble();
//                         }
//                         iter++;
//                     }

//                 }
//                 if(it.key() == "0C50DA25754C"){
//                     QJsonObject it_object = it.value().toObject();
//                     auto iter = it_object.begin();
//                     while(iter!=it_object.end())
//                     {
//                         if(iter.key() == "radian"){
//                             initpos(4) = iter.value().toDouble();
//                         }
//                         iter++;
//                     }

//                 }
//                 if(it.key() == "0850DA25754C"){
//                     QJsonObject it_object = it.value().toObject();
//                     auto iter = it_object.begin();
//                     while(iter!=it_object.end())
//                     {
//                         if(iter.key() == "radian"){
//                             initpos(5) = iter.value().toDouble();
//                         }
//                         iter++;
//                     }

//                 }
//                 it++;
//             }
//         }
//     }
// }
