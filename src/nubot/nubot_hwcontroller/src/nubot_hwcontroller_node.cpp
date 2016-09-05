#include <stdio.h>
#include <fstream>
#include <string>

// 终端
#include <ncurses.h>

// 系统中断
#include <unistd.h>
#include <sys/types.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sched.h>
#define DEFAULT_PRIO    80
using namespace boost::posix_time;

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

// C与C++混合编程要在CPP文件中加extern “C”关键字，否则链接会出错
#include "soem/EtherCAT.h"

// 动态参数服务器
#include <dynamic_reconfigure/server.h>
#include <nubot_hwcontroller/controllerConfig.h>

#include "nubot/nubot_hwcontroller/nubot_hwcontroller_node.h"
#include "nubot/nubot_hwcontroller/cmac.h"

#include "nubot_common/VelCmd.h"
#include "nubot_common/OdoInfo.h"
#include "nubot_hwcontroller/DebugInfo.h"
#include "nubot_common/Shoot.h"
#include "nubot_common/BallHandle.h"

#define CAN_SLOT_POSITION 4     // CAN模块位置
#define USE_BRUSH_MOTOR     0


#ifndef min
   #define min(x, y) (((x) > (y)) ? (y) : (x))
   #define max(x, y) (((x) > (y)) ? (x) : (y))
#endif

#define deg(a) (PI*(a)/180.0f)
#define Limmit(Vmin,v,Vmax)			(min(max((v),(Vmin)),(Vmax)))

int32 zero4[4]={0,0,0,0};
int32 zero2[2]={0,0};

/*********************DIY的计数器***************************
// Example:隔10次触发一次输出:
//       static trigger<10> t; if(t()){printf(...)}
// *******************************************************/
template <size_t N>
class trigger
{
public:
   trigger() :cnt(0){};

   size_t cnt;
   bool operator() ()
   {
       if (++cnt<N)
           return false;
       else
       {  cnt=0; return true; }
   }
};

#define USE_CURSE 1
Nubot_HWController::Nubot_HWController()
   :n(),number(0),motor_left(motor_speed[0]),motor_right(motor_speed[1]),
     Vx(0),Vy(0),w(0),P(30),I(0.2),D(2.4),Kp(10),Ki(0.2),Kd(1),
     cmac(1, 2, 20, 100, 0),
     BallHandleEnable(0),ShootEnable(false),shootcnt(0),ShootPos(-1),RodPos(0),ShootDone(true),
     PowerState(true),PowerState_Last(true),
     acc_state(0),wacc_state(0),
     RotationMode(0)
{
#if USE_CURSE
   initscr();
#endif
   // 获得机器人名字以及编号
   char hostname[65];
   gethostname(hostname, sizeof(hostname));
   name = hostname;
   sscanf(hostname,"nubot%d",&number);

   // 获得执行文件目录
   char exec_name[_POSIX_PATH_MAX];
   int result = readlink("/proc/self/exe", exec_name, _POSIX_PATH_MAX);

   // 获得配置文件所在目录，利用__FILE__宏定义和boost::filesystem获得源代码文件的目录位置
   boost::filesystem::path sourcefile(__FILE__);
   boost::filesystem::path dir(sourcefile.parent_path()/"../cfg"/hostname);
   if(!exists(dir))
       create_directory( dir );
   cfgdir=dir.c_str();cfgdir+="/";

   // 读取主动带球电位计的极限位置信息
   DribbleParamRead();

   // 初始化主动带球的小脑控制器CMAC
   double imin[3]={-500,-1,-1};
   double imax[3]={ 500, 1, 1};
   cmac.SetInputRange(imin,imax);
   cmac.LoadTable(cfgdir+"wtable.dat");

   // 初始化各个环形buffer
   e_l.assign(3,3,0); e_r.assign(3,3,0);
   Vxs.assign(2,2,0); Ws.assign(2,2,0);

   for(int i = 0 ;  i <  4; i++)
   {
       lastrel_v[i] = 0;
       rel_v[i] = 0;
   }

   // 初始化EtherCat和ELMO
   EtherCAT_init("eth0");

   // 注册里程计以及调试信息topic
   DebugInfo_pub = new realtime_tools::RealtimePublisher<nubot_hwcontroller::DebugInfo>(n, "/nubotdriver/debug", 1);
   OdeInfo_pub = new realtime_tools::RealtimePublisher<nubot_common::OdoInfo>(n, "/nubotdriver/odoinfo", 1);

   // 建立周期回调函数
   timer1=n.createTimer(ros::Rate(100),&Nubot_HWController::Timer1_Process,this);

   // 接受底盘速度指令
   Velcmd_sub_ = n.subscribe("/nubotcontrol/velcmd",1,&Nubot_HWController::SetSpeed,this);

   // 注册主动带球以及射门服务
   ballhandle_service_ = n.advertiseService("BallHandle",&Nubot_HWController::BallHandleControlService,this);
   shoot_service_ = n.advertiseService("Shoot",&Nubot_HWController::ShootControlServive,this);

   // 注册动态参数服务器
   f = boost::bind(&Nubot_HWController::ParamChanged, this, _1, _2);
   server.setCallback(f);
}

Nubot_HWController::~Nubot_HWController()
{
   Elmo_Process_2(zero2);
   Elmo_Process_1(zero4,0);

   Base_Enable(false);
   Ballhandle_Enable(false);

#if USE_CURSE
   endwin();
#endif
}

void Nubot_HWController::Run()
{
   ros::spin();
}

///////////////////////////////////////////////////////
/****************** HKH_201405 ***********************
//  功能: 初始化电位基准读数和量程两个参数
//  说明: 使用带球机构前需要确定电位器的最低和最高位置,
//		  最低位置对应完全没有碰到球的状态,最高位置对应
//		  球完全抱紧的状态.这两个参数取决于机构,比赛前
//		  需要进行标定.
//		  本函数负责执行这个标定功能,在比赛前应该先调
//		  用这个函数来确定上面说的两个参数,得到的参数会
//		  写在main目录下的DribbleParam.txt文件.这个函数
//		  也负责从这个文件读取参数
/*****************************************************/
#define  DribbleDebug 1
bool Nubot_HWController::DribbleParamRead(void)
{
   // 从文件读取之前测定的参数
//   ifstream param(cfgdir+"DribblePara.txt");
//   if(param)
//   {
//       param>>BallSensor_LBias				// 电位器的基准读数,对应没有球时候的角度
//           >>BallSensor_RBias
//           >>BallSensor_LStroke			// 电位器最大与最小读数的差值
//           >>BallSensor_RStroke;
//       param.close();
//       LeverPos_SetPoint= 0.7;
//       IsBallSensorInited = true;
//   }
//   else
       IsBallSensorInited = false;
   return IsBallSensorInited;
}
double
Nubot_HWController::LeverPosSet(int state, int last_state)
{

}

#define RADIUS 30.0f
void
Nubot_HWController::CompWithRoatate(double w, double vx)
{
  static double last_w = 0;
  double err = last_w - w;

  /*if(abs(err) > 0.5)
  {
     if(err >= 0)
         w = last_w - 0.2;
     else
         w = last_w + 0.2;
  }*/
  //vx;
  Vy = (1+2.0*(vx/250))*w*RADIUS;
  last_w = w;
}


bool cmac_enable=false;
const char* msg_state[]={"TryCatch","Firm","Unstable","Stuck"};
/****************** HKH_201405 ***********************
//	主动带球最底层的控制函数,根据电位器读数和车速进行反馈和前馈控制
//  入口参数:
//		FBRatio : 反馈量的权重,默认=1.0f,置0时相当于Disable反馈输出
//		FFRatio : 前馈量的权重,同上
//  备注:球完全抱紧时
//		原地旋转最大角速率 Wmax=2.0
//		最大侧移速度 Vy=50
********************************************************/


//饱和函数JK
double sat(double s)
{
  int deta=0.5;
  double M;
  {
      if (fabs(s)<=deta)
         M=s/deta;
     else
         M = s > 0 ? 1:-1;
  }
    return M;
}





void Nubot_HWController::DribbleController()
{

    if(!IsBallSensorInited)
       {
           // 没初始化跑个屁啊，先标定参数吧～
           DribbleParamCalibrate();
           return;
       }

       // 处理电位器数据
       DribbleGetState();

       if(!BallHandleEnable)
           return;

       typedef enum {TryCatch=0,Firm,Unstable,Stuck} BallStates;
       static BallStates state=TryCatch;

       static int LostCnt = 0, TightCnt = 0, FirmCnt=0, DetachCnt=0;

       bool LeftDetach  = LeverPos_Left<0.15;
       bool RightDetach = LeverPos_Right<0.15;
       LostCnt = (LeftDetach || RightDetach) ? LostCnt+1 : 0;
       state = LostCnt>30 ?  TryCatch : Firm;   // 延时切换丢球状态
    #if 0
       DetachCnt = (LeftDetach || RightDetach)? DetachCnt+1 : 0;
       FirmCnt= (LeverPos_Left>0.7&&LeverPos_Right>0.7)? FirmCnt+1 : 0;
       bool TooClose = LeverPos_Left>0.95 && LeverPos_Right>0.95;
       TightCnt= TooClose? TightCnt+1 : 0;
    #endif
       bool DoClutch     = false;
       bool DoFeedForward = (state!=TryCatch);
       bool DoIntergrate = false;

    /*
       switch(state)
       {
       //--------球完全脱离--------//
       case TryCatch:
           // 电机拼命转
           LeverPos_SetPoint = 3;
           //LeverPosSet = 3;
           // 抓紧了一段时间以后才切换状态
           if(FirmCnt>10)
           {
               state = Firm;
               FirmCnt = 0;
           }
           break;

       //--------球不稳定--------//
       case Unstable:
           // 电机拼命转
           LeverPos_SetPoint = 1.2;
           //LeverPosSet  =  3;
           // 抓紧了一段时间以后才切换状态
           if(FirmCnt>10)
           {
               state = Firm;
               FirmCnt = 0;
           }
           break;

       //--------正常运行--------//
       case Firm:

           switch(acc_state)
           {
           case 1: //正要加速
               LeverPos_SetPoint= 0.35;
               //LeverPosSet = 0.35;
               DoFeedForward = false;      // 关闭前馈，防止把球推出去
               break;

           case 0: //匀速
               // 在往前走球就离远点，在往后退球就离近点
                LeverPos_SetPoint= Vx>0? 0.35 : 0.75;
                // LeverPosSet = Vx>0? 0.35 : 0.75;;

               // 使能积分控制，消除静差
               if(wacc_state==0)
                   DoIntergrate = true;
               break;

           case -1: //刹车
               // 稍稍把球抱紧
               LeverPos_SetPoint = 0.9;
               //LeverPosSet = 3;

               // 关闭积分控制，防止把球推出去或抱死
               DoIntergrate = false;
               break;
           }

           state = DetachCnt>5? Unstable : state;
           break;

       //--------球堵住了--------//
       case Stuck:
           // 有持球的危险，调整一下
           LeverPos_SetPoint= Vx>0? 0.3 : 0.6;
           //LeverPosSet = Vx>0? 0.3 : 0.6;
           // 不再持球就正常工作
           if(!TooClose)
               state = Firm;
           break;

       default:
           state = TryCatch;
           break;
       }

          //if(LeverPosSet < LeverPos_SetPoint)
          //    LeverPos_SetPoint -= step;


       printw("State:%s (%.2f %.2f)\n",msg_state[state],Kx_f,Kx_b);
       printw("P: %.2f I: %.2f D:%.2f)\n",P,I,D);
    */
       const double VelRatio=75/105.0*1.414;   // 滚动轮线速度/球线速度,由机构决定
       const double WheelDiameter=5.2f;        // 主动轮半径,cm
       const double GearRatio=3.47f;           // 主动带球电机齿轮箱减速比
       const double K0 = -VelRatio/(M_PI*WheelDiameter);	// 车体速度cm/s 传递到带球电机转速(圈/秒)
       const float  LeverPos_Drible = 0.65;//带球三分之一时对应的电位计位置（电位计为0~1，放开为0，抓紧为1）

      // bool Rotating = w_diff>0.5;
       if( BallHandleEnable==2 )
           LeverPos_SetPoint =  2.0;
       else if( state == TryCatch)//RotationMode==1 ||
           LeverPos_SetPoint =  1.0;
       else
       {
         if(LeverPos_SetPoint>LeverPos_Drible+0.005)
           LeverPos_SetPoint -= 0.01;
         else if(LeverPos_SetPoint<LeverPos_Drible-0.005)
           LeverPos_SetPoint += 0.01;
       }

//       if(isturn)
//       {
//           LeverPos_SetPoint = 2.0;
           /*if(w>0)
               LeverPos_ErrL = LeverPos_ErrL+1.0;
           else
               LeverPos_ErrR = LeverPos_ErrR+1.0;*/
//       }

       LeverPos_ErrL = LeverPos_SetPoint - LeverPos_Left;
       LeverPos_ErrR = LeverPos_SetPoint - LeverPos_Right;

       // 用环形buffer记录历史误差，最新的误差索引总是为2，过去的数据索引依次递减
       e_l.push_back(LeverPos_ErrL);
       e_r.push_back(LeverPos_ErrR);
       printw("error_left:(%.1f) error_right:(%.1f)\n",e_l[2],e_r[2]);

    #if 0
       // 增量式PID，注意I参数不能为0，否则算法不起作用
       static double FeedBackVel_Left=0,FeedBackVel_Right=0;
       FeedBackVel_Left += P*(e_l[2]-e_l[1])+I*e_l[2]+D*(e_l[2]-2*e_l[1]+e_l[0]),
       FeedBackVel_Right+= P*(e_r[2]-e_r[1])+I*e_r[2]+D*(e_r[2]-2*e_r[1]+e_r[0]);
    #else
       // 绝对式PD
       static double ErrL_int=0, ErrR_int=0;
       ErrL_int  = DoIntergrate? ErrL_int+LeverPos_ErrL : 0;
       ErrR_int  = DoIntergrate? ErrR_int+LeverPos_ErrR : 0;

       double FeedBackVel_Left = e_l[2]*P + (e_l[2]-e_l[1])*D + I*ErrL_int,
              FeedBackVel_Right= e_r[2]*P + (e_r[2]-e_r[1])*D + I*ErrR_int;
    #endif

    #if 1
       // 异常情况处理:
       // 1.球没有同时接触两只爪爪
       if ( fabs(LeverPos_Diff)>0.3                // 两爪不平衡
            && (LeftDetach || RightDetach))		// 同时有一个爪空了
       {
           // 确保碰到球的那只爪不会往外推球
           FeedBackVel_Left  = (FeedBackVel_Left<0) ? 0 : FeedBackVel_Left;
           FeedBackVel_Right = (FeedBackVel_Right<0)? 0 : FeedBackVel_Right;

           // 将没有碰到球的那只爪的速度叠加到另外一只上
           if (LeverPos_Diff>0)
               FeedBackVel_Left += FeedBackVel_Right;    // 球在左边,往左边叠加
           else
               FeedBackVel_Right+= FeedBackVel_Left;     // 同理
       }
    #endif

       FeedBackVel_Left  = Limmit(-30,FeedBackVel_Left,30);
       FeedBackVel_Right = Limmit(-30,FeedBackVel_Right,30);

       /* 车速前馈控制,主要是对Vx的补偿,Vy和W的效果不好 */
       double FeedForwardVel_Left=0,FeedForwardVel_Right=0;
       if(DoFeedForward)
       {
    #if 0
           /*用小脑模型控制器学习前馈*/
           //if(cmac_enable && LeverPos_Left>0.2 && LeverPos_Right>0.2)
           {
               double input[3]={Vx,LeverPos_Left,LeverPos_Right};
               double output[2],delta[2];

               cmac.Calc(input,output);
               delta[0]=0.4*(FeedBackVel_Left -output[0]);
               delta[1]=0.4*(FeedBackVel_Right-output[1]);
               if(abs(LeverPos_ErrL) < 0.3 && abs(LeverPos_ErrR)<0.3)
                   cmac.Train(delta);

               FeedForwardVel_Left =output[0];
               FeedForwardVel_Right=output[1];
           }
    #else
           const double Kw = 3*K0;//RotationMode==1? 0 : 53*K0;
           const double Ky = 0.8*K0;
           //为了避免滞后，使用了期望值，为了避免超前，使用的实际值
           double feed_forward_x, feed_forward_y, feed_forward_w;
           if(Vx>Real_Vx)             feed_forward_x = Real_Vx;
           else                       feed_forward_x = Vx;
    //       if(fabs(Vy)>fabs(Real_Vy)) feed_forward_y = Real_Vy;
    //       else                       feed_forward_y = Vy;
    //       if(fabs(w)>fabs(Real_w))   feed_forward_w = Real_w;
    //       else                       feed_forward_w = w;
           feed_forward_y = (Vy+Real_Vy)/2;
           feed_forward_w = (w+Real_w)/2;
           // 前馈系数,前进与后退受力不同,故参数不同
           double Kx = feed_forward_x>0 ? Kx_f*K0 : Kx_b*K0;
           FeedForwardVel_Left  =feed_forward_x*Kx-feed_forward_y*Ky-feed_forward_w*30*Ky;
           FeedForwardVel_Right =feed_forward_x*Kx+feed_forward_y*Ky+feed_forward_w*30*Ky;
    #endif
       }
       // 控制器输出:主动带球电机转速(RPM)=反馈输出x反馈权重 + 前馈输出x前馈权重 */
       motor_left = (FeedBackVel_Left  + FeedForwardVel_Left )*80.0*GearRatio;
       motor_right= (FeedBackVel_Right + FeedForwardVel_Right)*80.0*GearRatio;

       //motor_left = (20  + FeedForwardVel_Left )*60.0*GearRatio;
       //motor_right= (-10 + FeedForwardVel_Right)*60.0*GearRatio;


       // 超过了最高速度elmo会报错
       motor_left = Limmit(-7000,motor_left, 7000);
       motor_right= Limmit(-7000,motor_right,7000);

       // reverting the right motor speed
       motor_right=-motor_right;
       Elmo_Process_2(motor_speed);
    //    BallLockEnable(DoClutch);

       printw("FeedBackVel:(%.1f %.1f) FeedForwardVel:(%.1f %.1f)\n",FeedBackVel_Left,FeedBackVel_Right,FeedForwardVel_Left,FeedForwardVel_Right);
       printw("Target:%.1f,Left:%.1f,Right:%.1f output:L:%d,R:%d\n",LeverPos_SetPoint,LeverPos_Left,LeverPos_Right,motor_left,motor_right);
    }

    inline void LeverUp()
    { EL2008_set(ShooterLever_Up_IO);   EL2008_clear(ShooterLever_Down_IO);
    }

    inline void LeverDown()
    { EL2008_clear(ShooterLever_Up_IO);  EL2008_set(ShooterLever_Down_IO);
    }

    inline void LeverStop()
    { EL2008_clear(ShooterLever_Up_IO);   EL2008_clear(ShooterLever_Down_IO);
    }

    void Nubot_HWController::ShooterController()
    {
       int RodPosReading = in_EL3064->invalue3;
       RodPos = (RodPosReading<13500)? -1 :
                (RodPosReading>23500)? 1 : 0;

       printw("RodPos:%d CMD:%d\n",RodPosReading,ShootPos);

       if(RodPos==0 || RodPos!=ShootPos)
       {
           switch(ShootPos)
           {
           case -1:
               LeverDown();break;
           case 1:
               LeverUp();break;
           default:
               LeverStop();
               break;
           }
       }
       else
           LeverStop();
}

bool Nubot_HWController::ShootControlServive(nubot_common::Shoot::Request  &req,
                        nubot_common::Shoot::Response &res)
{
   // Whether to shoot or not
   if(req.strength==0)
   {
       // Adjust the shooter rod only
       ShootPos   = req.ShootPos;
       res.ShootIsDone = (RodPos==ShootPos)&&RodPos!=0? 1 : 0;
   }
   else if(ShootDone==true)
   {
       ShootDone == false;
       ShootPower = req.strength;

       // 确保已经射完了才重新开始
       boost::thread t1(boost::bind(&Nubot_HWController::Process_Shoot, this));
       res.ShootIsDone = ShootDone? 1:0;
   }
   else
       res.ShootIsDone = 0;

   return true;
}

void Nubot_HWController::ParamChanged(nubot_hwcontroller::controllerConfig &config, uint32_t level)
{
   //cmac_enable=config.Use_CMAC;
   P = config.FeedBack_P;
   D = config.FeedBack_D;
   I = config.FeedBack_I;
   Kx_f=config.Kx_f;
   Kx_b=config.Kx_b;
   //LeverPos_SetPoint = config.Target_Position;
}

const double WHEEL_DISTANCE=20.3;
const double MOTOR_GEAR_RATIO=21.0;
const double WHEEL_DIAMETER=12.0;
const double VEL_TO_RPM_RATIO=MOTOR_GEAR_RATIO*60.0/(M_PI*WHEEL_DIAMETER);

void Nubot_HWController::Timer1_Process(const ros::TimerEvent &e)
{
#if USE_CURSE
   clear();
#endif
   ros::Duration time_err=e.current_real-e.current_expected;
   printw("Time error:%.2fms,Last duration:%.2fms\n",time_err.toSec()*1000,(e.profile.last_duration).toSec()*1000);

   // 断电重启
   if(PowerState_Last==false && PowerState==true)
       EtherCAT_init("eth0");
   PowerState_Last = PowerState;

   printw("Vx:%.1f,\tVy:%.1f,\tw:%.1f \tAcc:%d WAcc:%d w/v:%.4f\n",Vx,Vy,w,acc_state,wacc_state,w/(Vx+0.1));

   //Elmo_Process_error();
   //Elmo_Process_enable();

#if 1
   static trigger<2> t;
   if(t())
   {
       BaseController();

       // 发送里程计信息
       if (OdeInfo_pub->trylock())
       {
           OdeInfo_pub->msg_.header.stamp = ros::Time::now();
           OdeInfo_pub->msg_.w =Real_w;
           OdeInfo_pub->msg_.Vx=Real_Vx;
           OdeInfo_pub->msg_.Vy=Real_Vy;
           OdeInfo_pub->msg_.PowerState=PowerState;
           OdeInfo_pub->msg_.RobotStuck = RobotStuck;
           OdeInfo_pub->unlockAndPublish();
       }
   }
#endif
 DribbleController();

 // 发送状态信息
   if (DebugInfo_pub->trylock())
   {
       DebugInfo_pub->msg_.d1 =w_diff;
       DebugInfo_pub->msg_.d2 =RotationMode;
       DebugInfo_pub->unlockAndPublish();
   }

   ShooterController();
   // 判断动力电是否断开
   PowerState = in_EL3064->invalue4 > 10000;
   printw("Power is %s Shootcnt:%d\n",PowerState? "On":"Off",shootcnt);

#if USE_CURSE
  refresh();
#endif
}

bool Nubot_HWController::BallHandleControlService(nubot_common::BallHandle::Request  &req,
                             nubot_common::BallHandle::Response &res)
{
   //
   if(BallHandleEnable!=req.enable)
       Ballhandle_Enable(req.enable);
   BallHandleEnable=req.enable;

   //
   res.BallIsHolding = BallSensor_IsHolding;

   return true;
}

const double LIMITEDRPM=12000;
void Nubot_HWController::BaseController(/*const ros::TimerEvent &event*/)
{
   int32 v[4], vmax,Real_v[7];



    static ros::Time start = ros::Time::now();
           ros::Duration time = ros::Time::now() - start;

    static  std::ofstream local_velocity("/home/nubot4/yang_xianglin/telejoy/time_local_cmd_real.txt");

    static  std::ofstream wheel_velocity("/home/nubot4/yang_xianglin/telejoy/time_wheel_cmd_real.txt");


//    static  std::ofstream local_velocity("/home/nubot4/yang_xianglin/coach/time_local_cmd_real.txt");

//    static  std::ofstream wheel_velocity("/home/nubot4/yang_xianglin/coach/time_wheel_cmd_real.txt");



//   Vx+=0.4*(Vx_cmd-Vx);
//   Vy+=1*(Vy_cmd-Vy);
//   w +=0.4*(w_cmd-w);
   Vx=Vx_cmd; Vy=Vy_cmd; w=w_cmd;

 //  ROS_INFO("%.1f  %.1f  %.1f ",Vx,Vy,w);



//   if(RotationMode==1 && BallSensor_IsHolding == 1)
//       Vy-=w*30;

#if USE_BRUSH_MOTOR
   //  有刷电机
   v[0]= ( -0.707*Vy + 0.707*Vx - w*WHEEL_DISTANCE)*-VEL_TO_RPM_RATIO;
   v[1]= (  0.707*(Vx + Vy)     - w*WHEEL_DISTANCE)*-VEL_TO_RPM_RATIO;
   v[2]= (  0.707*(-Vx + Vy)    - w*WHEEL_DISTANCE)*-VEL_TO_RPM_RATIO;
   v[3]= ( -0.707*Vx - 0.707*Vy - w*WHEEL_DISTANCE)*-VEL_TO_RPM_RATIO;
#else
   //  无刷电机
   v[0]= ( -0.707*Vy + 0.707*Vx - w*WHEEL_DISTANCE)*-VEL_TO_RPM_RATIO;
   v[1]= (  0.707*(Vx + Vy)     - w*WHEEL_DISTANCE)*-VEL_TO_RPM_RATIO;
   v[2]= (  0.707*(-Vx + Vy)    - w*WHEEL_DISTANCE)*-VEL_TO_RPM_RATIO;
   v[3]= ( -0.707*Vx - 0.707*Vy - w*WHEEL_DISTANCE)*-VEL_TO_RPM_RATIO;


 //  ROS_INFO("%d  %d  %d  %d",v[0],v[1],v[2],v[3]);


#endif
   vmax = abs(v[0]);
   for(int i  = 1; i <  4; i++)
          if(abs(v[i]) >  vmax)
             vmax = abs(v[i]);

   if(vmax > LIMITEDRPM)
       for(int i  = 0 ; i <  4; i++)
            v[i] = v[i]*LIMITEDRPM/vmax;

   Elmo_Process_1(v,Real_v);
   //printf("M1: %d  M2: %d  M3: %d  M4: %d  M5: %d  M6: %d  M7: %d\n",Real_v[0],Real_v[1],Real_v[2],Real_v[3],Real_v[4],Real_v[5],Real_v[6]);


//#if USE_BRUSH_MOTOR
   for(int i=0;i<4;i++)
       Real_v[i]=-Real_v[i];
//#endif
   rel_v[0] = Real_v[0];
   rel_v[1] = Real_v[1];
   rel_v[2] = Real_v[2];
   rel_v[3] = Real_v[3];


   int derr;
   if(rel_v[0] == 0 || rel_v[1] == 0 || rel_v[2] == 0 || rel_v[3] == 0)
   {
       for(int i = 0; i <  4; i++)
       {
         derr = sqrt((rel_v[i]-lastrel_v[i])*(rel_v[i]-lastrel_v[i]));

         if(derr > 200)
         {
             for(int i = 0; i <  4; i++)
             {
               Real_v[i] = lastrel_v[i];
               rel_v[i] = lastrel_v[i];
             }
         }

         break;
       }
   }

   for(int i  = 0; i <  4; i++)
      lastrel_v[i] = rel_v[i];



   Real_w  = (-Real_v[1] - Real_v[3])/(2*WHEEL_DISTANCE*VEL_TO_RPM_RATIO);
   Real_Vx = (Real_v[0]+Real_v[1]-Real_v[2]-Real_v[3])/(2*1.414*VEL_TO_RPM_RATIO);
   Real_Vy = (Real_v[1]+Real_v[2]-Real_v[0]-Real_v[3])/(2*1.414*VEL_TO_RPM_RATIO);

   double squared_err1 =(v[0]-Real_v[0])*(v[0]-Real_v[0]);
   double squared_err2 =(v[1]-Real_v[1])*(v[1]-Real_v[1]);
   double squared_err3 =(v[2]-Real_v[2])*(v[2]-Real_v[2]);
   double squared_err4 =(v[3]-Real_v[3])*(v[3]-Real_v[3]);

   double squared_err = squared_err1 + squared_err2 + squared_err3 + squared_err4;


   RobotStuck = DetectingRobotStuck(squared_err);



  local_velocity<<time.toSec()<<" "
          <<Vx_cmd<<" "<<Vy_cmd<<" "<<w_cmd<<" "
          <<Real_Vx<<" "<<Real_Vy<<" "<< Real_w<<"\n";


  wheel_velocity<<time.toSec()<<" "
          <<v[0]<<" "<<v[1]<<" "<<v[2]<<" "<<v[3]<<" "
          << Real_v[0]<<" "<< Real_v[1]<<" "<< Real_v[2]<<" "<< Real_v[3]<<"\n";



}

bool Nubot_HWController::DetectingRobotStuck(double squared_err)
{
  bool rtvl = false;
  double expvel = sqrt(Vx*Vx+Vy*Vy);
  double actvel = sqrt(Real_Vx*Real_Vx+ Real_Vy*Real_Vy);
  double exp = Vx/(Vy+1);
  double act = Real_Vx/(Real_Vy+1);
  static int  robotstuckcnt = 0;
  static bool robotstuckflg = false;


  if(expvel > 30)
  {
      if( squared_err >  2*LIMITEDRPM && (robotstuckcnt == 0 || robotstuckflg == true))
      {
          robotstuckcnt++;
          robotstuckflg = true;
      }
      else
      {
          robotstuckcnt = 0;
          robotstuckflg = false;

      }

      rtvl = robotstuckcnt > 75 ? true : false;
  }
  else
  {
      robotstuckcnt = 0;
      robotstuckflg = false;
      rtvl = false;
  }
  return rtvl;
}
void Nubot_HWController::SetSpeed(const nubot_common::VelCmd::ConstPtr& cmd)
{
   Vx_cmd=cmd->Vx;
   Vy_cmd=cmd->Vy;
   w_cmd =cmd->w;

   Vxs.push_back(Vx_cmd);
   Ws.push_back(w_cmd);

   // 判断车体运动状态
   // 车体最大物理加速度250cm/s^2, 角加速度10.8rad/s^2
   Vx_diff = (Vxs[1]-Vxs[0])*33;

   w_diff = abs(Ws[1]-Ws[0])/M_PI;

   switch (RotationMode) {
   //  原地旋转
   case 0:
       // 角加速度大于0.3PI的时候就甩屁股，防止丢球
      if(w_diff>0.25)
        RotationMode=1;
       break;

   // 以球为中心甩屁股
   case 1:
      // 车静止以后恢复原地转模式
      if(abs(w)<0.05)
        RotationMode=0;
       break;
   default:
       RotationMode=0;
       break;
   }

}


/****************** HKH_201405 ***********************
//  功能: 标定主动带球电位器最高和最低读数
//  说明: 本函数为non-blocking函数，你需要按一定周期调用本函数，
//     当函数返回true时表示标定已完成
/*****************************************************/
bool Nubot_HWController::DribbleParamCalibrate(void)
{
   static int state=0;
   static int cnt=0;
   static float sum_l=0,sum_r=0;

   const int NUM=120;

   // 获得电位器读数
   unsigned int    BallHandlingSensor_Left = in_EL3064->invalue1;
   unsigned int    BallHandlingSensor_Right= in_EL3064->invalue2;

#if DribbleDebug
   //static ofstream dbg("drible.txt");
  // static trigger<10> t;
  // if(t())
       printw("L:%d	R:%d",in_EL3064->invalue1,in_EL3064->invalue2);
#endif

   IsBallSensorInited = false;
   switch (state)
   {
   case 0:// 统计电位器的基准电压
       if(cnt<NUM)
       {
           if ( BallHandlingSensor_Left != 0 )
           {
               sum_l += (float)BallHandlingSensor_Left;
               sum_r += (float)BallHandlingSensor_Right;
               cnt++;
           }
       }

       //已经采集了足够的数据
       else
       {
           // 对30个数据求平均来实现滤波
           BallSensor_LBias = sum_l/NUM;					// 基准电压
           BallSensor_RBias = sum_r/NUM;
#if DribbleDebug
           printw("LBias:%.3f	RBias:%.3f",BallSensor_LBias,BallSensor_RBias);
#endif
           //跳转到下个状态
           state = 1;
           cnt = 0;
           sum_l = sum_r = 0.0f;
       }
       motor_left=motor_right=0;
       break;

   case 1:// 驱动电机，记录完全持球时的最高电压
       if(cnt<(NUM+10))
       {
           // 还没抓到球，使劲抓球
           if (  BallHandlingSensor_Left > BallSensor_LBias+20
               &&BallHandlingSensor_Right > BallSensor_RBias+20)
           {
               if (cnt>9)
               {
                   sum_l += (float)BallHandlingSensor_Left;
                   sum_r += (float)BallHandlingSensor_Right;
               }
               cnt++;
               motor_left = 600; motor_right= -motor_left;
           }
            // 已经抓到球，减慢滚球速度
           else
           {
               motor_left = 300; motor_right= -motor_left;
           }
       }
       else  // 已经采集了足够的数据
       {
           // 对数据求平均来滤波
           float BallSensor_LMax = sum_l/NUM;
           float BallSensor_RMax = sum_r/NUM;

           if (BallSensor_LMax>BallSensor_LBias && BallSensor_RMax>BallSensor_RBias)
           {
               BallSensor_LStroke = BallSensor_LMax - BallSensor_LBias; // 电压范围=max-min
               BallSensor_RStroke = BallSensor_RMax - BallSensor_RBias;
               IsBallSensorInited = true;
#if DribbleDebug
               printw("LStroke:%.3f	RStroke:%.3f",BallSensor_LStroke,BallSensor_RStroke);
#endif
               ofstream out(cfgdir+"DribblePara.txt");
               out<<BallSensor_LBias<<' '
                   <<BallSensor_RBias<<' '
                   <<BallSensor_LStroke<<' '
                   <<BallSensor_RStroke;
               out.close();
               LeverPos_SetPoint=0.7;
               printw("Calibration done!");
               motor_left=motor_right=0;
           }
           else  //采到的数据不合格，重来
           {
               cnt = 0;
               sum_l = sum_r = 0.0f;
           }
       }
       break;
   default:
       break;
   }

   Elmo_Process_2(motor_speed);

   return IsBallSensorInited;
}


/****************** HKH_201405 ***********************
//  功能: 通过电位器读数获得带球机构的状态
//  说明: 本函数对原始AD读数进行归一化和滤波,并更新和
//		  返回相关的状态变量
/*****************************************************/
void Nubot_HWController::DribbleGetState(void)
{
   // 获得电位器读数
   int BallSensor_Left = in_EL3064->invalue1 -BallSensor_LBias;
   int BallSensor_Right= in_EL3064->invalue2 -BallSensor_RBias;

   // 将电位器读数归一化,0对应完全没碰到球,1对应球完全进来了
   LeverPos_Left  = BallSensor_Left /BallSensor_LStroke;
   LeverPos_Right = BallSensor_Right/BallSensor_RStroke;

   printw("L0:%.1f(%d),	R0:%.1f(%d)\n",
          LeverPos_Left,BallSensor_Left+(int)BallSensor_LBias,
          LeverPos_Right,BallSensor_Right+(int)BallSensor_RBias);

   static ofstream out("/home/nubot4/JK/activedribble.txt");   //JK
   out<<LeverPos_Left <<" "
   <<LeverPos_Right<<"\n";


   // 限幅,防止偶发的小于0或大于1的情况
   LeverPos_Left = Limmit(0.0f, LeverPos_Left,  1.0f);
   LeverPos_Right= Limmit(0.0f, LeverPos_Right, 1.0f);

   // 左右球距的偏差
   LeverPos_Diff = LeverPos_Left - LeverPos_Right;

   static int HoldingCnt = 0, UnholdingCnt = 0;

   HoldingCnt =  (LeverPos_Left>0.5&&LeverPos_Right>0.5)? HoldingCnt+1 : 0;

   UnholdingCnt =  (LeverPos_Left <0.05&&LeverPos_Right<0.05)? UnholdingCnt+1 : 0;

   if( !BallSensor_IsHolding && HoldingCnt > 10 )
      BallSensor_IsHolding = true;
   else if( BallSensor_IsHolding && UnholdingCnt > 10 )
      BallSensor_IsHolding = false;
}



#define  Delay2IGBT     100   //继电器导通到IGBT导通控制时间间隔,一次1ms,共100ms
#define  ShotFinishTime 500  //射门完成周期,一次1ms,共500ms
void Nubot_HWController::Process_Shoot()
{
   ShootDone = false;
   ptime start= microsec_clock::local_time();
   ptime now  = microsec_clock::local_time();

   time_duration diff = now - start;

   while(diff < microsec(Delay2IGBT*1000) )
   {
       EL2008_set(Relay_IO);

       //时间还没够
       time_duration dt=min(microsec(20*1000), microsec(Delay2IGBT*1000)-diff);
       usleep(dt.total_microseconds());

       diff = microsec_clock::local_time() - start;
       //std::cout<<diff.total_milliseconds()<<"-OpenRelay"<<endl;
   }

   while(diff < microsec(Delay2IGBT*1000+ShootPower*1000) )
   {
       EL2008_set(Relay_IO);
       EL2008_set(IGBT_IO);

       //时间还没够
       time_duration dt=min(microsec(20*1000), microsec(Delay2IGBT*1000+ShootPower*1000)-diff);
       usleep(dt.total_microseconds());

       diff = microsec_clock::local_time() - start;
       //std::cout<<diff.total_milliseconds()<<"-OpenIGBT"<<endl;
   }

   EL2008_clear(Relay_IO);
   EL2008_clear(IGBT_IO);
   ShootDone = true,        shootcnt ++;
   //std::cout<<"-Done"<<endl;
}

void Nubot_HWController::BallLockEnable(bool on)
{
   if(on)
   {
       EL2008_set(BallLock_Left_IO);
       EL2008_set(BallLock_Right_IO);
   }
   else
   {
       EL2008_clear(BallLock_Left_IO);
       EL2008_clear(BallLock_Right_IO);
   }
}


int main(int argc,char** argv)
{
    struct sched_param schedp;
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = DEFAULT_PRIO;
   if (sched_setscheduler(0, SCHED_FIFO, &schedp)) {
      printf("set scheduler failed.\n");
      sched_setscheduler(0, SCHED_OTHER, &schedp);
   } 
   ros::init(argc,argv,"nubot_hwcontroller_node");

   Nubot_HWController controller;

   controller.Run();

   return 0;
}






