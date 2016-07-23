#ifndef BALL_HANDLE_H
#define BALL_HANDLE_H

#include <ros/ros.h>
#include <boost/circular_buffer.hpp>

#include <boost/asio.hpp>

#include <realtime_tools/realtime_publisher.h>
#include "nubot_common/VelCmd.h"
#include "nubot_common/OdoInfo.h"
#include "nubot_common/Shoot.h"
#include "nubot_common/BallHandle.h"
#include "nubot/nubot_hwcontroller/cmac.h"

// 动态参数服务器
#include <dynamic_reconfigure/server.h>
#include <nubot_hwcontroller/controllerConfig.h>
#include "nubot_hwcontroller/DebugInfo.h"


using namespace std;

class Nubot_HWController
{
public:
    Nubot_HWController();
    ~Nubot_HWController();
    bool DribbleParamRead(void);
    bool DribbleParamCalibrate(void);
    void DribbleGetState(void);
    void DribbleController();
    void CompWithRoatate(double w,double vx);
    void ShooterController();
    void Timer1_Process(const ros::TimerEvent &event);
    void BallLockEnable(bool on);
    void Shoot_Control(uint8 *IsKick, float32 ShootPower);
    bool DetectingRobotStuck(double squared_err);
    void SetSpeed(const nubot_common::VelCmd::ConstPtr& cmd);
    void ParamChanged(nubot_hwcontroller::controllerConfig &config, uint32_t level);
    void BaseController(/*const ros::TimerEvent &event*/);
    void Process_Shoot();
    void Run();

    bool BallHandleControlService(nubot_common::BallHandle::Request  &req,
                                  nubot_common::BallHandle::Response &res);
    bool ShootControlServive(nubot_common::Shoot::Request  &req,
                             nubot_common::Shoot::Response &res);

    double LeverPosSet(int state, int last_state);
private:
    ros::NodeHandle n;
    ros::Timer timer1;

    // 动态参赛服务器
    dynamic_reconfigure::Server<nubot_hwcontroller::controllerConfig> server;
    dynamic_reconfigure::Server<nubot_hwcontroller::controllerConfig>::CallbackType f;

    realtime_tools::RealtimePublisher<nubot_hwcontroller::DebugInfo> *DebugInfo_pub;
    realtime_tools::RealtimePublisher<nubot_common::OdoInfo> *OdeInfo_pub;

    ros::Subscriber Velcmd_sub_;

    ros::ServiceServer ballhandle_service_;
    ros::ServiceServer shoot_service_;

public:
    int   BallHandleEnable;
    bool  ShootEnable;
    float ShootPower;
    int   ShootPos,RodPos;
    int   ShootState,shootcnt;
    bool  ShootDone;

    bool  PowerState,PowerState_Last;
    bool  RobotStuck;

    string name;        // 机器人名字
    string cfgdir;      // 配置文件所在目录
    int number;         // 机器人编号，nubotXXX

    CMAC  cmac;
    //double cmac_ip[3],cmac_op[2],cmac_delta[2];

    int32 motor_speed[2];
    int32 &motor_left,&motor_right;
    int32 Motor_Readings[4];

    double Vx,Vy,w,Vx_diff,w_diff;
    double Vx_cmd,Vy_cmd,w_cmd;
    int RotationMode;
    int acc_state,wacc_state;
    double Real_Vx,Real_Vy,Real_w;
    double FBRatio,FFRatio;
    double P,I,D;
    double Kp,Ki,Kd;
    double Kx_f,Kx_b;

    bool isturn;

    // PID控制器的反馈误差历史
    boost::circular_buffer<double> e_l,e_r;
    double LeverPos_ErrL,LeverPos_ErrR;
    // 速度指令历史
    boost::circular_buffer<double> Vxs,Ws;


    double LeverPos_Left,LeverPos_Right,LeverPos_Diff;
    double LeverPos_SetPoint;
    double BallSensor_LBias,BallSensor_RBias,BallSensor_LStroke,BallSensor_RStroke;
    bool IsBallSensorInited;
    bool BallSensor_IsHolding,IsBallLost;
    int rel_v[4];
    int lastrel_v[4];

};


#endif
