/*
 * nubot_teleop_key.cpp
 *
 *  Created on: 2012-10-11
 *      Author: hkh
 */
#include <ros/ros.h>
#include <stdio.h>
#include <pthread.h>

#include <math.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <nubot_common/VelCmd.h>
#include <nubot_common/OminiVisionInfo.h>
#include <nubot_common/BallHandle.h>
#include <nubot_common/Shoot.h>


class TeleopNubot
{
public:
	TeleopNubot();
	ros::NodeHandle n;
    ros::ServiceClient ballhandle_client_;
    ros::ServiceClient shoot_client_;

    double Vx,Vy,w;

private:
	ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber ball_sub;

    nubot_common::VelCmd cmd;
    bool CatchEnable;
    double ball_angle;
	double deadzone_;

	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void ballCallback(const nubot_common::OminiVisionInfo::ConstPtr& ball);
};

TeleopNubot::TeleopNubot()
    :Vx(0),Vy(0),w(0),ball_angle(0),CatchEnable(false)
{
    vel_pub = n.advertise<nubot_common::VelCmd>("/nubotcontrol/velcmd", 10);
    joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 2, &TeleopNubot::joyCallback, this);
    ball_sub= n.subscribe<nubot_common::OminiVisionInfo>("/omnivision/OminiVisionInfo", 2, &TeleopNubot::ballCallback, this);
    ballhandle_client_ =  n.serviceClient<nubot_common::BallHandle>("BallHandle");
    shoot_client_ = n.serviceClient<nubot_common::Shoot>("Shoot");
}

// 手柄消息回调函数，在ros::spin()里执行
#define idx_X 1
#define idx_Y 0
#define idx_w 3

#define Button_A    0
#define Button_B    1
#define Button_X    2
#define Button_Y    3

#define Button_Up   5
#define Button_Down 4

void TeleopNubot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    static float strength = 1;
    //if(joy->buttons[Button_Up] || joy->buttons[Button_Down])
    {
        if(joy->buttons[Button_A])
        {
            /*nubot_common::Shoot s;
            s.request.ShootPos=0;
            s.request.strength=17;
            shoot_client_.call(s);*/
            strength += 0.2;

            if(strength > 40)
               strength = 40;

        }
        else if(joy->buttons[Button_X])
        {
            /*nubot_common::Shoot s;
            s.request.ShootPos=0;
            s.request.strength=8;
            shoot_client_.call(s);*/
            strength -= 0.2;

            if(strength < 0)
               strength = 0;

        }
        else if(joy->buttons[Button_Y])
        {
            nubot_common::Shoot s;
            s.request.ShootPos=0;
            s.request.strength=strength;
            shoot_client_.call(s);
        }
        else
        {
            nubot_common::Shoot s;
            s.request.strength=0;
            s.request.ShootPos= joy->buttons[Button_Up]  ?    1 :
                                joy->buttons[Button_Down]?    -1 : 0;
            shoot_client_.call(s);
        }
    }
    ROS_INFO("POWER :  %.1f", strength);
    static bool BallHandleEnable = false;
    if(joy->buttons[Button_B]==1)
    {
        BallHandleEnable=!BallHandleEnable;
        nubot_common::BallHandle b;
        b.request.enable=BallHandleEnable;
        ballhandle_client_.call(b);
    }

    /*速度指令*/
    cmd.Vx = joy->axes[idx_X]*500;
    cmd.Vy = joy->axes[idx_Y]*500;

    CatchEnable = joy->buttons[Button_X];
    // 朝球转
    //if(!CatchEnable)
        cmd.w  = joy->axes[idx_w]*2*M_PI;

        //*yqh ACC method
        #define WHEELS 4
        const double WHEEL_DISTANCE=20.3;
        #define ACC_THRESH 15  //cm/s^2
        #define DCC_THRESH (ACC_THRESH*3)  //cm/s^2
           //同时平动和转动，要想保持全局速度方向不变，会给轮子带来额外的加速度开销，暂且称为牵连加速度:|acc_convect| <= |v*w|
          static float wheel_speed_old[WHEELS] = {0};
          float wheel_speed[WHEELS];
          float wheel_acc[WHEELS];
          float& Vx = cmd.Vx;
          float& Vy = cmd.Vy;
          float&  w = cmd.w;

        if(WHEELS == 4)
        {
          wheel_speed[0]= ( 0.707*( Vx - Vy) - w*WHEEL_DISTANCE);
          wheel_speed[1]= ( 0.707*( Vx + Vy) - w*WHEEL_DISTANCE);
          wheel_speed[2]= ( 0.707*(-Vx + Vy) - w*WHEEL_DISTANCE);
          wheel_speed[3]= ( 0.707*(-Vx - Vy) - w*WHEEL_DISTANCE);
        }
        else
        {
            wheel_speed[0]= ( 0.866*Vx -  0.5*Vy - w*WHEEL_DISTANCE);
            wheel_speed[1]= (   0.0*Vx +      Vy - w*WHEEL_DISTANCE);
            wheel_speed[2]= ( -0.866*Vx - 0.5*Vy - w*WHEEL_DISTANCE);
        }
          float acc_thresh_ratio = 1;
          for(int i=0; i<WHEELS; i++)
          {
            wheel_acc[i] = wheel_speed[i]-wheel_speed_old[i];
            float acc_thresh_ratio_temp = 0;
            if( wheel_acc[i]*wheel_speed_old[i]>=0 ) //speed up
              acc_thresh_ratio_temp = fabs(wheel_acc[i])/ACC_THRESH;
            else                                 //speed down
              acc_thresh_ratio_temp = fabs(wheel_acc[i])/DCC_THRESH;
            if( acc_thresh_ratio_temp>acc_thresh_ratio )
            {
               acc_thresh_ratio = acc_thresh_ratio_temp;
            }
          }

          if( acc_thresh_ratio > 1 )
          {
            for(int i=0; i<WHEELS; i++)
            {
              wheel_acc[i] /= acc_thresh_ratio;
              wheel_speed[i] = wheel_speed_old[i] + wheel_acc[i];
            }
          }
          if(WHEELS==4)
          {
            w  = -(wheel_speed[0]+wheel_speed[1]+wheel_speed[2]+wheel_speed[3])/(4*WHEEL_DISTANCE);
            Vx =  (wheel_speed[0]+wheel_speed[1]-wheel_speed[2]-wheel_speed[3])/(2*1.414);
            Vy =  (wheel_speed[1]+wheel_speed[2]-wheel_speed[0]-wheel_speed[3])/(2*1.414);
          }
          else
          {
            Vx = ( 0.577*wheel_speed[0]  + 0 * wheel_speed[1] -  wheel_speed[2] * 0.577);
            Vy = (-0.333*wheel_speed[0]  + 0.667*wheel_speed[1] - wheel_speed[2]*0.333);
            w  = (-wheel_speed[0] - wheel_speed[1] - wheel_speed[2] )/(3*WHEEL_DISTANCE);
          }

          if(hypot(Vx,Vy)*fabs(w)*0.03>ACC_THRESH)//无法保持全局速度方向不变,进一步限速
          {
            float v_wheel=0;
            for(int i=0; i<WHEELS; i++)
            {
              if( fabs(wheel_speed[i])>v_wheel )
                v_wheel = fabs(wheel_speed[i]);
            }
            if(v_wheel<ACC_THRESH) v_wheel = ACC_THRESH;
            for(int i=0; i<WHEELS; i++)
            {
              wheel_speed[i] *= (1-ACC_THRESH/v_wheel);
            }
            if(WHEELS==4)
            {
              w  = -(wheel_speed[0]+wheel_speed[1]+wheel_speed[2]+wheel_speed[3])/(4*WHEEL_DISTANCE);
              Vx =  (wheel_speed[0]+wheel_speed[1]-wheel_speed[2]-wheel_speed[3])/(2*1.414);
              Vy =  (wheel_speed[1]+wheel_speed[2]-wheel_speed[0]-wheel_speed[3])/(2*1.414);
            }
            else
            {
              Vx = ( 0.577*wheel_speed[0]  + 0 * wheel_speed[1] -  wheel_speed[2] * 0.577);
              Vy = (-0.333*wheel_speed[0]  + 0.667*wheel_speed[1] - wheel_speed[2]*0.333);
              w  = (-wheel_speed[0] - wheel_speed[1] - wheel_speed[2] )/(3*WHEEL_DISTANCE);
            }
          }
 //         float Vx_r = m_plan_.m_behaviour_.worldmodelinfo_.robot_vel_.x_;
 //         float Vy_r = m_plan_.m_behaviour_.worldmodelinfo_.robot_vel_.y_;
 //         float  w_r = m_plan_.m_behaviour_.worldmodelinfo_.robot_omega_;
//          float temp = Vx;
//          Vx -=-Vy*w*0.1;
//          Vy -= temp*w*0.1;

          for(int i=0; i<WHEELS; i++)
          {
            wheel_speed_old[i] = wheel_speed[i];
          }



    vel_pub.publish(cmd);
}

void TeleopNubot::ballCallback(const nubot_common::OminiVisionInfo::ConstPtr& ball)
{
    ball_angle = ball->ballinfo.real_pos.angle;
    ROS_INFO("Angle:%.2f",ball_angle/M_PI*180.0);

    if(CatchEnable)
    {
        cmd.w  = ball_angle;
        vel_pub.publish(cmd);
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nubot_teleop_joy");
	TeleopNubot teleop_nubot;

#if 1
	// 调用joy_node进程,驱动手柄
	pid_t pid = vfork();
	if(pid==0)
	{
        if(execlp("rosrun", "rosrun", "joy", "joy_node", "_deadzone:=0.14", (char *)0) <0)
			ROS_WARN("Process Joy not found!");

		// 正常情况下exec函数不会返回
		return(-1);
	}
#endif

    ros::spin();

	return (0);
}


