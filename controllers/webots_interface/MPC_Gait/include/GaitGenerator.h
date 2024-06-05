#ifndef GAITGENERATOR_H_
#define GAIGAITGENERATOR_H_
// #include "Footplanner.h"
// #include "LIP_x_MPC.h"
// #include "LIP_y_MPC.h"
// #include "SLIP_z_MPC.h"
// #include "../../MPC_Landing/include/Landing_mpc.h"
// #include "Landing_mpc.h"
#include "state.h"
#include "XFsmMgr.h"
#include "../../SonnyRobotController/include/Robot_Data.h"
#include "../../SonnyRobotController/include/Robot_Controller.h"
// #include "Robot_Data.h"
// #include "Robot_Controller.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>
/**
 * @file GaitGenerator.h
 * @author ren xiaoyu 
 * @brief generate the move commands,for example: point to point, tracking a desired speed, tracking a path, etc
 * @version 0.1
 * @date 2022-03-29
 * 
 * @copyright Copyright (c) 2022
 * 
 * 
 */
class GaitGenerator
{
    public:
    GaitGenerator();
    ~GaitGenerator();
    void init(QString path, // robotdata json
            QString path2, // mpc json
            double _dt,
            DataPackage * data);
    // the FSM running
    void gait_run(DataPackage * data_);
    // set event
    void setevent(string event_);
    void set_current_fsm_command(string current_command);
    // set desired speed
    void setvelocity(double vx,double vy, double vyaw);
    void setvelocity_offset(double vx_offest, double vy_offset);
    // set desired posiition
    void setxyz(double x, double y, double z);
    void setrollpitch(double roll, double pitch, double yaw);
    // robotarm
    double getleftamp();
    double getleftphase();
    double getrightamp();
    double getrightphase();
    	// for test
    // interp fifth
    void FifthPoly(Eigen::VectorXd p0, Eigen::VectorXd p0_dot, Eigen::VectorXd p0_dotdot,     // start point states 
                     Eigen::VectorXd p1, Eigen::VectorXd p1_dot, Eigen::VectorXd p1_dotdot,     // end point states 
                     double totalTime,       // total permating time 
                     double currenttime,     //current time,from 0 to total time 
                     Eigen::VectorXd& pd, Eigen::VectorXd& pd_dot, Eigen::VectorXd& pd_dotdot );// output command     
    void FifthPoly(double p0, double p0_dot, double p0_dotdot,     // start point states 
                     double p1, double p1_dot, double p1_dotdot,     // end point states 
                     double totalTime,       // total permating time 
                     double currenttime,     //current time,from 0 to total time 
                     double& pd, double& pd_dot, double& pd_dotdot );// output command 
    void Thirdpoly(double p0, double p0_dot,double p1, double p1_dot,
                    double totalTime,       // total permating time 
                     double currenttime,     //current time,from 0 to total time 
                     double& pd, double& pd_dot); 
    void LinePoly(double p0,double p1,
                    double totalTime,       // total permating time 
                     double currenttime,     //current time,from 0 to total time 
                     double& pd, double& pd_dot); 
    // excitingtrajectory
    void ExcitingTrajectoryFunction(Eigen::VectorXd  series_para,  
                            Eigen::VectorXd init_pos,
                            int dof_num,
                            int series_num,
                            double base_freq,
                            double curtime,
                            Eigen::VectorXd& pos, 
                            Eigen::VectorXd& vel,
                            Eigen::VectorXd& acc);

    //unit Quaternion interpolation
    Eigen::Matrix3d QuanteiniontoMatrix(RigidBodyDynamics::Math::Quaternion Q);
    void QuaternionInterp(Eigen::Matrix3d R_start,
                        Eigen::Matrix3d R_end,
                        double totaltime, double currenttime,
                        Eigen::Matrix3d& R_d, Eigen::Vector3d& omiga_d, Eigen::Vector3d& acc_d);


    // robot data
    Robot_Controller robot_controller_;
    string fsmstatename;
    // walk2stand
    bool W2S = false;
    string event;
    //
    string current_fsmstate_command;
    private:
    // state machine
    XFsmMgr fsm;
    // string event;
    Step* stepstate;
    Jump* jumpstate;
    ExcitingTrajectory* excitingtrajectory;
    MotorMotion* motormotion;
    Start* startstate;
    Zero* zerostate;
    Stand* standstate;
    Walk* walkstate;
    SingleLegTest* singlelegtest;
    SingleSwingTest* singleswingsest;
    S2W* S2Wstate;
    Z2S* Z2Sstate;
    Squat* Squatstate;
    Stop* Stopstate;
    int initFsm();

    // set arm cmd
    double leftarm_phase = 0.5;
    double leftarm_amp = 1.0;
    double rightarm_phase = 0.5;
    double rightarm_amp = 1.0;

};
#endif
