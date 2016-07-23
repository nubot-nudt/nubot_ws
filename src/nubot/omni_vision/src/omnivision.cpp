#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/transfer.h"
#include "nubot/omni_vision/scanpoints.h"
#include "nubot/omni_vision/whites.h"
#include "nubot/omni_vision/optimise.h"
#include "nubot/omni_vision/glocalization.h"
#include "nubot/omni_vision/odometry.h"
#include "nubot/omni_vision/localization.h"
#include "nubot/omni_vision/obstacles.h"
#include "nubot/omni_vision/colorsegment.h"
#include "nubot/omni_vision/ballfinder.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

#include <nubot_common/OdoInfo.h>
#include <nubot_common/RobotInfo.h>
#include <nubot_common/BallInfo.h>
#include <nubot_common/ObstaclesInfo.h>
#include <nubot_common/OminiVisionInfo.h>
#include <omni_vision/OmniVisionConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sched.h>
#define DEFAULT_PRIO    80
using namespace cv;
using namespace nubot;

namespace encodings=sensor_msgs::image_encodings;
namespace nubot {

struct RobotInformation
{
    DPoint2d realvtrans_;
    DPoint2d worldvtrans_;
    DPoint   visual_location_;
    DPoint   final_location_;
    Angle    angle_;
    double   angular_velocity_;
    bool     isglobal_;
};

class Omni_Vision
{

public:

    OmniImage        * imginfo_;
    Transfer         * tranfer_;
    ScanPoints       * scanpts_;
    Whites           * whites_;
    Optimise         * optimise_;
    Globallocalization * glocation_;
    Odometry         * odometry_;
    Localization     * location_;
    Obstacles        * obstacles_;
    BallFinder       * ball_finder_;
    ColorSegment     * colorsegment_;
    FieldInformation field_info_;

    RobotInformation robot;


    image_transport::Subscriber img_sub_;
    ros::Subscriber motor_info_sub_;

    ros::Publisher  ballinfo_pub_;
    ros::Publisher  robotinfo_pub_;
    ros::Publisher  obstaclesinfo_pub_;
    ros::Publisher  omin_vision_pub_;

    nubot_common::BallInfo        ball_info_;
    nubot_common::RobotInfo       robot_info_;
    nubot_common::ObstaclesInfo   obstacles_info_;
    nubot_common::OminiVisionInfo omin_vision_info_;
    dynamic_reconfigure::Server<omni_vision::OmniVisionConfig> reconfigureServer_;

    bool is_show_ball_;
    bool is_show_whites_;
    bool is_show_obstacles_;
    bool is_show_scan_points;
    bool is_show_result_;

    bool is_restart_;
    bool is_robot_stuck_;
    bool is_power_off_;
    cv::Mat field_image_;

    ros::Time receive_time_;
    int Agent_ID_;
public:

    Omni_Vision(int argc, char **argv)
    {
        char * environment;
        if((environment = getenv("AGENT"))==NULL)
        {
            ROS_ERROR("this agent number is not read by robot");
            return ;
        }
        Agent_ID_ = atoi(environment);
        std::stringstream ss;
        ss<<Agent_ID_;
        std::string calibration_path="/home/nubot"+ss.str()+"/nubot_ws/src/nubot/omni_vision/calib_results";
        if(argc>1)
            calibration_path=argv[1];

        ROS_INFO("initialize the omni_vision  process");
        imginfo_     = new OmniImage(calibration_path+"/"+ss.str()+"/ROI.xml");
        tranfer_     = new Transfer(calibration_path+"/"+ss.str()+"/mirror_calib.xml",*imginfo_);
        scanpts_     = new ScanPoints(*imginfo_);
        whites_      = new Whites(*scanpts_,*tranfer_);
        optimise_    = new Optimise(calibration_path+"/errortable.bin",
                                    calibration_path+"/Diff_X.bin",
                                    calibration_path+"/Diff_Y.bin",
                                    *tranfer_);
        glocation_   = new Globallocalization(*optimise_);
        odometry_    = new Odometry();
        location_    = new Localization(*optimise_);
        obstacles_   = new Obstacles(*scanpts_,*tranfer_);
        ball_finder_ = new BallFinder(*tranfer_);
        colorsegment_= new ColorSegment(calibration_path+"/"+ss.str()+"/CTable.dat");

        is_show_ball_=false;
        is_show_obstacles_=false;
        is_show_whites_=false;
        is_show_scan_points=false;
        is_show_result_=false;
        is_robot_stuck_ = false;

        robot.isglobal_=true;
        is_restart_=false;
        is_power_off_=false;
        if(WIDTH_RATIO<1)
            field_image_ = cv::imread(calibration_path+"/"+ss.str()+"/field_mine.bmp");
        else
            field_image_ = cv::imread(calibration_path+"/"+ss.str()+"/field.bmp");

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        img_sub_= it.subscribe("/camera/image_raw", 1, &Omni_Vision::imageCallback,this);
        ros::NodeHandle local_nh;
        motor_info_sub_ = local_nh.subscribe("/nubotdriver/odoinfo", 1, &Omni_Vision::odometryupdate,this);

        ros::NodeHandle node;
        omin_vision_pub_   = node.advertise<nubot_common::OminiVisionInfo>("/omnivision/OmniVisionInfo",1);
        reconfigureServer_.setCallback(boost::bind(&Omni_Vision::configure, this, _1, _2));
    }
    ~Omni_Vision()
    {


    }

public:

    void
    configure(const omni_vision::OmniVisionConfig & config, uint32_t level)
    {
        ROS_INFO("Reconfigure request received");
        is_show_ball_       = config.ball;
        is_show_whites_     = config.white;
        is_show_obstacles_  = config.obstacle;
        is_show_scan_points = config.scan;
        is_show_result_     = config.show;
        int  obstacle_thres = config.obsthres;
        double obstacle_length_thres = config.obs_length_thres;
        double obstacle_basic_thres = config.obs_basic_thres;
        obstacles_->setObsThres(obstacle_thres,obstacle_length_thres,obstacle_basic_thres);
        ROS_INFO("Reconfigure request end");
    }

    void
    publish()
    {
        robot_info_.header.stamp = receive_time_;
        robot_info_.header.seq++;
        robot_info_.heading.theta = robot.angle_.radian_;
        robot_info_.pos.x         = robot.final_location_.x_;
        robot_info_.pos.y         = robot.final_location_.y_;
        robot_info_.vtrans.x      = robot.worldvtrans_.x_;
        robot_info_.vtrans.y      = robot.worldvtrans_.y_;
        robot_info_.vrot          = robot.angular_velocity_;
        robot_info_.AgentID       = Agent_ID_;
        robot_info_.isstuck       = is_robot_stuck_;
        robot_info_.isvalid       = is_power_off_;
        if(is_show_result_)
        {
            ROS_INFO("pos(%.f, %.f, %d, %.1f, %.1f, %.1f, %d ,%d, %d)",
                     robot.final_location_.x_,robot.final_location_.y_,robot.angle_.degree(),robot.worldvtrans_.x_,
                     robot.worldvtrans_.y_,robot_info_.vrot,whites_->img_white_.size(),robot.isglobal_,is_power_off_);
        }

        ball_info_.header.stamp = receive_time_;
        ball_info_.header.seq++;
        ball_info_.pos.x =  ball_finder_->get_ball_global_loc().x_;
        ball_info_.pos.y =  ball_finder_->get_ball_global_loc().y_;
        ball_info_.real_pos.angle  = ball_finder_->get_ball_real_loc().angle_.radian_;
        ball_info_.real_pos.radius = ball_finder_->get_ball_real_loc().radius_;
        if(is_show_result_)
        {
            ROS_INFO("ball(%.f, %.f, %.1f, %.1f ,%d ,%d)",ball_info_.pos.x,ball_info_.pos.y, ball_info_.velocity.x,
                     ball_info_.velocity.y,ball_info_.pos_known, ball_finder_->ball_area_.area_size_);
        }
        obstacles_info_.header.stamp= ros::Time::now();
        obstacles_info_.header.seq++;
        obstacles_info_.pos.clear();
        obstacles_info_.polar_pos.clear();
        int length= obstacles_->real_obstacles_.size();
        nubot_common::Point2d point;
        nubot_common::PPoint  polar_point;
        for(int i = 0 ; i < length ; i++)
        {
            DPoint & pt=obstacles_->world_obstacles_[i];
            PPoint & polar_pt= obstacles_->real_obstacles_[i];
            point.x=pt.x_;
            point.y=pt.y_;
            polar_point.angle=polar_pt.angle_.radian_;
            polar_point.radius=polar_pt.radius_;
            obstacles_info_.pos.push_back(point);
            obstacles_info_.polar_pos.push_back(polar_point);
        }
        if(is_show_result_)
        {
            for(int i = 0 ;i < OBS_VALUABLE_NUMBER ;i++)
            {
                if (i<obstacles_info_.pos.size())
                    ROS_INFO("obs_omni(%.f, %.f)",obstacles_info_.pos[i].x,obstacles_info_.pos[i].y);
                //  else
                //       ROS_INFO("obs_omni(%.f, %.f)",-10000.0,-10000.0);
            }
        }
        omin_vision_info_.header.stamp = robot_info_.header.stamp;
        omin_vision_info_.header.seq++;
        omin_vision_info_.ballinfo=ball_info_;
        omin_vision_info_.obstacleinfo=obstacles_info_;
        omin_vision_info_.robotinfo.clear();
        omin_vision_info_.robotinfo.push_back(robot_info_);
        omin_vision_pub_.publish(omin_vision_info_);
    }

    void
    process()
    {
        if(!colorsegment_->Segment(imginfo_->yuv_image_))
            return;

        //get the colors of the scan_points
        scanpts_->process();
        //detect the white points
        whites_->process();

        if(whites_->img_white_.size()<=0)
        {
            //  is_restart_=true;
            ROS_WARN("don't detect the white points");
            return ;
        }
        if(!is_power_off_)
            is_restart_=true;
        if(is_restart_)
        {
            is_restart_=false;
            robot.isglobal_=true;
        }
        //localization
        if(robot.isglobal_)
        {
            ROS_INFO("start global localization");
            robot.isglobal_=glocation_->process(whites_->weights_,whites_->robot_white_,robot.visual_location_,robot.angle_);
        }
        else
        {
            DPoint delta_loc=odometry_->getWorldLocaton();
            Angle  delta_ang=odometry_->getDeltaAngle();
            odometry_->clear(robot.angle_);
            robot.realvtrans_       = odometry_->getRealVelocity();
            robot.worldvtrans_      = odometry_->getWorldVelocity();
            robot.angular_velocity_ = odometry_->getAngularVelocity();
            location_->process(whites_->weights_,whites_->robot_white_,robot.visual_location_,robot.angle_,delta_loc,delta_ang);
        }
        PPoint correct_pt = PPoint(tranfer_->get_offset().angle(),tranfer_->get_offset().norm());
        tranfer_->calculateWorldCoordinates(correct_pt,robot.visual_location_,robot.angle_,robot.final_location_);
        obstacles_->process(colorsegment_->segment_result_,robot.final_location_,robot.angle_);
        ball_info_.pos_known=ball_finder_->Process(colorsegment_->segment_result_,robot.final_location_,robot.angle_);
        if(is_show_whites_ || is_show_obstacles_ || is_show_ball_ )
        {
            if(field_image_.empty())
                ROS_INFO("Field.bmp is empty");
            cv::Mat image  = field_image_.clone();
            cv::Mat orgnal = imginfo_->getBGRImage().clone();
            static double length = 1920;
            static double width  = 1314;
            if(WIDTH_RATIO<1)
            {
                length = 1920;
                width  = 882;
            }
            if(is_show_scan_points)
                scanpts_->showScanPoints();

            if(is_show_obstacles_)
            {
                obstacles_->showObstacles(orgnal);
                if(!field_image_.empty())
                    obstacles_->showObstacles(image,robot.final_location_,robot.angle_,length,width);
            }
            if(is_show_whites_)
            {
                whites_->showWhitePoints(orgnal);
                if(!field_image_.empty())
                    whites_->showWhitePoints(image,robot.final_location_,robot.angle_,length,width);
            }
            if(is_show_ball_)
            {
                ball_finder_->showBall(orgnal);
                if(!field_image_.empty())
                    ball_finder_->showBall(image,robot.final_location_,robot.angle_,length,width);
            }
        }
    }

    void
    imageCallback(const sensor_msgs::ImageConstPtr& _image_msg)
    {
        ros::Time start = ros::Time::now();
        receive_time_ = _image_msg->header.stamp;
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr=cv_bridge::toCvShare(_image_msg,encodings::BGR8);
        Mat orignalimage=cv_ptr->image;
        bool isthreechannels=imginfo_->setBGRImage(orignalimage);
        if(!isthreechannels)
        {
            ROS_WARN("the image doesn't have three channels");
            return ;
        }
        imginfo_->bgr2yuv();
        process();
        publish();
        //   ROS_INFO("start omni_vision imageCallback");
        static ros::Time time_before = ros::Time::now();
        ros::Duration duration  = ros::Time::now() - time_before;
        ros::Duration duration1 = ros::Time::now() - start;
        time_before = ros::Time::now();
        ROS_INFO("omni_time: %d %d %d",int(1.0/duration.toSec()),int(1.0/duration1.toSec()),whites_->img_white_.size());
    }

    void
    odometryupdate(const nubot_common::OdoInfo & _motorinfo_msg)
    {
        static std::vector<double> motor_data(nubot::MOTOR_NUMS_CONST,0);
        static ros::Time time_before = _motorinfo_msg.header.stamp;
        ros::Time time_update = _motorinfo_msg.header.stamp;
        motor_data[0]=(double)_motorinfo_msg.Vx;
        motor_data[1]=(double)_motorinfo_msg.Vy;
        motor_data[2]=(double)_motorinfo_msg.w;
        is_power_off_   = _motorinfo_msg.PowerState;
        is_robot_stuck_ = _motorinfo_msg.RobotStuck;
        ros::Duration duration= time_update-time_before;
        time_before = time_update ;
        double secs=duration.toSec();
        odometry_->process(motor_data,secs);
    }


};


}// end of namespace nubot


int main(int argc, char **argv)
{ 
    struct sched_param schedp;
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = DEFAULT_PRIO;
   if (sched_setscheduler(0, SCHED_FIFO, &schedp)) {
      printf("set scheduler failed.\n");
      sched_setscheduler(0, SCHED_OTHER, &schedp);
   }
    ros::init(argc,argv,"omni_vision_node");
    ros::Time::init();
    ROS_INFO("start omni_vision process");
    nubot::Omni_Vision vision_process(argc, argv);
    ros::spin();
    return 0;
}
