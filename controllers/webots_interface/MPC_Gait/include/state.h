#ifndef STATE_H
#define STATE_H

#include <iostream>
#include "XFsmState.h"
#include <Eigen/Dense>
#include <fstream>
#include "../../WalkPlan/include/aeroWalkPlan.h"
#include "../../WalkPlan/include/gaitPlan.h"

class Start : public XFsmState {
public:
    Start(void* App) : XFsmState(App) { };
    void onEnter();
    void run();
    void onExit();
    void init();
};

class Zero : public XFsmState {
public:
    Zero(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	// current joint states
	Eigen::MatrixXd qa;
	Eigen::MatrixXd qa_dot;
	// target joint states
	Eigen::MatrixXd qd;
	Eigen::MatrixXd qd_dot;
	// current joint states
	Eigen::MatrixXd qaWaist;
	Eigen::MatrixXd qa_dotWaist;
	// target joint states
	Eigen::MatrixXd qdWaist;
	Eigen::MatrixXd qd_dotWaist;
	// total time
	double totaltime;
	// average speed
	double avr_v;
		//data log
	std::ofstream foutData;
	Eigen::VectorXd dataL;

	Eigen::VectorXd xStand, qCmd, qDotCmd;
	bool footflag;
};

class Z2S : public XFsmState {
public:
    Z2S(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	// com task: 
	// Eigen::MatrixXd com_x_a;
	// Eigen::MatrixXd com_x_d;
	// Eigen::MatrixXd com_x_c;
	// body orientation task: 
	Eigen::MatrixXd body_x_a;
	Eigen::MatrixXd body_x_d;
	Eigen::MatrixXd body_x_c;
	// left foot task:
	Eigen::MatrixXd left_foot_x_a;
	Eigen::MatrixXd left_foot_x_d;
	Eigen::MatrixXd left_foot_x_c;
	// right foot task:
	Eigen::MatrixXd right_foot_x_a;
	Eigen::MatrixXd right_foot_x_d;
	Eigen::MatrixXd right_foot_x_c;
	// total time
	double totaltime;
	// average speed
	double avr_v;
			//data log
	std::ofstream foutData2;
	Eigen::VectorXd dataL2;

	Eigen::VectorXd xStand, qCmd, qDotCmd;
	Eigen::VectorXd xStand_tgt;
	bool first_flag;
};


class Stand : public XFsmState {
public:
    Stand(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	// com task: 
	// Eigen::MatrixXd com_x_a;
	// Eigen::MatrixXd com_x_d;
	// Eigen::MatrixXd com_x_c;
	// body orientation task: 
	Eigen::MatrixXd body_x_a;
	Eigen::MatrixXd body_x_d;
	Eigen::MatrixXd body_x_c;
	// left foot task:
	Eigen::MatrixXd left_foot_x_a;
	Eigen::MatrixXd left_foot_x_d;
	Eigen::MatrixXd left_foot_x_c;
	// right foot task:
	Eigen::MatrixXd right_foot_x_a;
	Eigen::MatrixXd right_foot_x_d;
	Eigen::MatrixXd right_foot_x_c;
	// total time
	double totaltime;
	// average speed
	double avr_v;
			//data log
	std::ofstream foutData2;
	Eigen::VectorXd dataL2;

	Eigen::VectorXd xStand, qCmd, qDotCmd;
	Eigen::VectorXd xStand_tgt;
	Eigen::VectorXd xStand_Zero;

	Eigen::VectorXd q_factor_init;
	Eigen::VectorXd q_dot_factor_init;
	bool first_flag;
	Eigen::Vector3d torso_d;
	Eigen::VectorXd rTorsoCmd;
	Eigen::VectorXd rFootCmd;
};

class S2W : public XFsmState {
public:
    S2W(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	// com task: 
	// Eigen::MatrixXd com_x_a;
	// Eigen::MatrixXd com_x_d;
	// Eigen::MatrixXd com_x_c;
	// body orientation task: 
	Eigen::MatrixXd body_x_a;
	Eigen::MatrixXd body_x_d;
	Eigen::MatrixXd body_x_c;
	// left foot task:
	Eigen::MatrixXd left_foot_x_a;
	Eigen::MatrixXd left_foot_x_d;
	Eigen::MatrixXd left_foot_x_c;
	// right foot task:
	Eigen::MatrixXd right_foot_x_a;
	Eigen::MatrixXd right_foot_x_d;
	Eigen::MatrixXd right_foot_x_c;
	// total time
	double totaltime;
	// average speed
	double avr_v;
			//data log
	std::ofstream foutData2;
	Eigen::VectorXd dataL2;

	Eigen::VectorXd xStand, qCmd, qDotCmd;
	Eigen::VectorXd xStand_tgt;
	bool first_flag;
	Eigen::VectorXd rTorsoCmd;
	Eigen::VectorXd rFootCmd;
};


class Walk : public XFsmState {
public:
    Walk(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	// 
	Eigen::VectorXd q_init;
	// 
	Eigen::VectorXd q_c;
	Eigen::VectorXd q_dot_c;
	Eigen::VectorXd q_ddot_c;
	Eigen::VectorXd tau_c;
	//data log
	std::ofstream foutData;
	Eigen::VectorXd dataL;
	//
	// bool dataLog(Eigen::VectorXd &v, std::ofstream &f);

	//intialize bezier parameter
    int M;
    int Nout;
    int stIndex;
    double tStepPre;
    Eigen::VectorXd init_pos;
    
    //WorkSpace Trajectory
    bool firstFlag;
    int Nwout;
    Eigen::VectorXd xInit, xDotInit, xDDotInit;
    Eigen::VectorXd xEnd, xDotEnd;
    Eigen::VectorXd xCmd, xDotCmd, xDDotCmd, fCmd;
	Eigen::VectorXd qCmd, qDotCmd;
    double vCmd;
    Eigen::VectorXd xStand;
	gaitPlan *gait_plan;
	// wbc
	// body orientation task: 
	Eigen::MatrixXd body_x_a;
	Eigen::MatrixXd body_x_d;
	Eigen::MatrixXd body_x_c;
	// left foot task:
	Eigen::MatrixXd left_foot_x_a;
	Eigen::MatrixXd left_foot_x_d;
	Eigen::MatrixXd left_foot_x_c;
	// right foot task:
	Eigen::MatrixXd right_foot_x_a;
	Eigen::MatrixXd right_foot_x_d;
	Eigen::MatrixXd right_foot_x_c;
	// prepare to stand
	// bool prestand = false;
	int prestand_step = 0;
	Eigen::VectorXd rTorsoCmd;
	Eigen::VectorXd rFootCmd;
};

class Stop : public XFsmState {
public:
    Stop(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	// current joint states
	Eigen::MatrixXd qa;
	Eigen::MatrixXd qa_dot;
	// target joint states
	Eigen::MatrixXd qd;
	Eigen::MatrixXd qd_dot;
	// total time
	double totaltime;
	// average speed
	double avr_v;
		//data log
	std::ofstream foutData;
	Eigen::VectorXd dataL;

	Eigen::VectorXd xStand, qCmd, qDotCmd;
	bool footflag;
};


class MotorMotion : public XFsmState {
public:
    MotorMotion(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	double dt = 0.0025;
	// motor number
	int motorNum;
	double Tduration;
	// current joint states
	Eigen::MatrixXd qa;
	Eigen::MatrixXd qa_dot;
	// target joint states
	Eigen::MatrixXd qCmd;
	Eigen::MatrixXd qDotCmd;
	Eigen::VectorXd vmax;
	Eigen::VectorXd vdelt;
	Eigen::Matrix<int,Eigen::Dynamic,1> N;
	// total time
	double totaltime;
	// average speed
	double avr_v;
		//data log
	std::ofstream foutData;
	Eigen::VectorXd dataL;
};

class ExcitingTrajectory : public XFsmState {
public:
    ExcitingTrajectory(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer;
	int dof_num;
  	int series_num;
  	double base_freq;
  	int total_num;
  	Eigen::VectorXd init_pos;  
	std::ifstream fin_data;
  	Eigen::VectorXd series_para; 
	Eigen::VectorXd pos_;
	Eigen::VectorXd vel_;
	Eigen::VectorXd acc_;
			//data log
	std::ofstream foutData;
	Eigen::VectorXd dataL;
};

class Step : public XFsmState {
public:
    Step(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	// com task: 
	Eigen::MatrixXd com_x_a;
	Eigen::MatrixXd com_x_d;
	Eigen::MatrixXd com_x_c;
	// body orientation task: 
	Eigen::MatrixXd body_x_a;
	Eigen::MatrixXd body_x_d;
	Eigen::MatrixXd body_x_c;
	// left foot task:
	Eigen::MatrixXd left_foot_x_a;
	Eigen::MatrixXd left_foot_x_d;
	Eigen::MatrixXd left_foot_x_c;
	// right foot task:
	Eigen::MatrixXd right_foot_x_a;
	Eigen::MatrixXd right_foot_x_d;
	Eigen::MatrixXd right_foot_x_c;
	// total time
	double totaltime;
	// average speed
	double avr_v;
	Eigen::MatrixXd qa;
	Eigen::MatrixXd qa_dot;

};


// class Stand : public XFsmState {
// public:
//     Stand(void* App) : XFsmState(App) { };
// 	void onEnter();
// 	void run();
// 	void onExit();
// 	void init();
// 	// clock
// 	double timer = 0.0;
// 	// com task: 
// 	Eigen::MatrixXd com_x_a;
// 	Eigen::MatrixXd com_x_d;
// 	Eigen::MatrixXd com_x_c;
// 	// body orientation task: 
// 	Eigen::MatrixXd body_x_a;
// 	Eigen::MatrixXd body_x_d;
// 	Eigen::MatrixXd body_x_c;
// 	// left foot task:
// 	Eigen::MatrixXd left_foot_x_a;
// 	Eigen::MatrixXd left_foot_x_d;
// 	Eigen::MatrixXd left_foot_x_c;
// 	// right foot task:
// 	Eigen::MatrixXd right_foot_x_a;
// 	Eigen::MatrixXd right_foot_x_d;
// 	Eigen::MatrixXd right_foot_x_c;
// 	// total time
// 	double totaltime;
// 	// average speed
// 	double avr_v;
// };

// class Walk : public XFsmState {
// public:
// 	Walk(void* app) : XFsmState(app){};
// 	void onEnter();
// 	void run();
// 	void onExit();
// 	void init();
// 	// com task: 
// 	Eigen::MatrixXd com_x_a;
// 	Eigen::MatrixXd com_x_d;
// 	Eigen::MatrixXd com_x_c;
// 	// body orientation task: 
// 	Eigen::MatrixXd body_x_a;
// 	Eigen::MatrixXd body_x_d;
// 	Eigen::MatrixXd body_x_c;
// 	// left foot task:
// 	Eigen::MatrixXd left_foot_x_a;
// 	Eigen::MatrixXd left_foot_x_d;
// 	Eigen::MatrixXd left_foot_x_c;
// 	// right foot task:
// 	Eigen::MatrixXd right_foot_x_a;
// 	Eigen::MatrixXd right_foot_x_d;
// 	Eigen::MatrixXd right_foot_x_c;
// 	// clock
// 	double timer = 0.0;
// 	// support leg
// 	double supportleg;
// };

class Jump : public XFsmState {
public:
    Jump(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	// com task: 
	Eigen::MatrixXd com_x_a;
	Eigen::MatrixXd com_x_d;
	Eigen::MatrixXd com_x_c;
	// body orientation task: 
	Eigen::MatrixXd body_x_a;
	Eigen::MatrixXd body_x_d;
	Eigen::MatrixXd body_x_c;
	// left foot task:
	Eigen::MatrixXd left_foot_x_a;
	Eigen::MatrixXd left_foot_x_d;
	Eigen::MatrixXd left_foot_x_c;
	// right foot task:
	Eigen::MatrixXd right_foot_x_a;
	Eigen::MatrixXd right_foot_x_d;
	Eigen::MatrixXd right_foot_x_c;
	// total time
	double totaltime;
	// average speed
	double avr_v;
	// flag stamp
	bool flag_go_down;
	bool flag_go_up;
	bool flag_flight;
	bool flag_landing;
	//fifth
		// var for fifth
	Eigen::VectorXd P0;
	Eigen::VectorXd P0_dot;
	Eigen::VectorXd P0_ddot;
	Eigen::VectorXd P1;
	Eigen::VectorXd P1_dot;
	Eigen::VectorXd P1_ddot;
	Eigen::VectorXd Pd;
	Eigen::VectorXd Pd_dot;
	Eigen::VectorXd Pd_ddot;
	// delt left pos right pos
	Eigen::MatrixXd deltleftpos;
	Eigen::MatrixXd deltrightpos;
	// joint pos
	Eigen::MatrixXd q;
	Eigen::MatrixXd q_dot;
	double v0_1;
	double v0_1_a;
	double v0_1_aa;
};



class SingleLegTest : public XFsmState {
public:
    SingleLegTest(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	// 
	Eigen::VectorXd q_init;
	// 
	Eigen::VectorXd q_c;
	Eigen::VectorXd q_dot_c;
	Eigen::VectorXd q_ddot_c;
	Eigen::VectorXd tau_c;
	//data log
	std::ofstream foutData;
	Eigen::VectorXd dataL;
	//
	// bool dataLog(Eigen::VectorXd &v, std::ofstream &f);
};

class SingleSwingTest : public XFsmState {
public:
    SingleSwingTest(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	// 
	Eigen::VectorXd q_init;
	// 
	Eigen::VectorXd q_c;
	Eigen::VectorXd q_dot_c;
	Eigen::VectorXd q_ddot_c;
	Eigen::VectorXd tau_c;
	//data log
	std::ofstream foutData;
	Eigen::VectorXd dataL;
	//
	// bool dataLog(Eigen::VectorXd &v, std::ofstream &f);

	//intialize bezier parameter
    int M;
    int Nout;
    int stIndex;
    double tStepPre;
    Eigen::VectorXd init_pos;
    
    //WorkSpace Trajectory
    bool firstFlag;
    int Nwout;
    Eigen::VectorXd xInit, xDotInit, xDDotInit;
    Eigen::VectorXd xEnd, xDotEnd;
    Eigen::VectorXd xCmd, xDotCmd, xDDotCmd, fCmd;
	Eigen::VectorXd qCmd, qDotCmd;
    double vCmd;
    Eigen::VectorXd xStand;
	gaitPlan *gait_plan;
	// wbc
	// body orientation task: 
	Eigen::MatrixXd body_x_a;
	Eigen::MatrixXd body_x_d;
	Eigen::MatrixXd body_x_c;
	// left foot task:
	Eigen::MatrixXd left_foot_x_a;
	Eigen::MatrixXd left_foot_x_d;
	Eigen::MatrixXd left_foot_x_c;
	// right foot task:
	Eigen::MatrixXd right_foot_x_a;
	Eigen::MatrixXd right_foot_x_d;
	Eigen::MatrixXd right_foot_x_c;
};

class Squat : public XFsmState {
public:
    Squat(void* App) : XFsmState(App) { };
	void onEnter();
	void run();
	void onExit();
	void init();
	// clock
	double timer = 0.0;
	// com task: 
	// Eigen::MatrixXd com_x_a;
	// Eigen::MatrixXd com_x_d;
	// Eigen::MatrixXd com_x_c;
	// body orientation task: 
	Eigen::MatrixXd body_x_a;
	Eigen::MatrixXd body_x_d;
	Eigen::MatrixXd body_x_c;
	// left foot task:
	Eigen::MatrixXd left_foot_x_a;
	Eigen::MatrixXd left_foot_x_d;
	Eigen::MatrixXd left_foot_x_c;
	// right foot task:
	Eigen::MatrixXd right_foot_x_a;
	Eigen::MatrixXd right_foot_x_d;
	Eigen::MatrixXd right_foot_x_c;
	// total time
	double totaltime;
	// average speed
	double avr_v;
			//data log
	std::ofstream foutData2;
	Eigen::VectorXd dataL2;

	Eigen::VectorXd xStand, qCmd, qDotCmd;
	Eigen::VectorXd xStand_tgt;
	Eigen::VectorXd xStand_Zero;

	Eigen::VectorXd q_factor_init;
	Eigen::VectorXd q_dot_factor_init;
	bool first_flag;
	Eigen::Vector3d torso_d;
};


#endif
