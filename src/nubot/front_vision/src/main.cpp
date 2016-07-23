#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <nubot_common/FrontBallInfo.h>
#include <nubot/front_vision/colorsegment.h>
#include <nubot/front_vision/ballfinder.h>

using namespace cv;
using namespace nubot;

namespace encodings=sensor_msgs::image_encodings;
namespace nubot {

class Front_Vision
{

private:
    image_transport::Subscriber img_sub_;
    ColorSegment * colorsegmet_;
    BallFinder   * ballfinder_;

    ros::Publisher  ballinfo_pub_;
    nubot_common::FrontBallInfo ball_info_;

public:

Front_Vision(int argc, char **argv)
{
    char * environment;
    int agent;
    if((environment = getenv("AGENT"))==NULL)
    {
        ROS_ERROR("this agent number is not read by robot");
        return ;
    }
    agent = atoi(environment);
    std::stringstream ss;
    ss<<agent;
    std::string calibration_path="/home/nubot"+ss.str()+"/nubot_ws/src/nubot/front_vision/calib_results/"+ss.str();

    if(argc>1)
        calibration_path=argv[1];
    ROS_INFO("initialize the front_vision  process");

    colorsegmet_ = new ColorSegment(calibration_path+"/CTableBGR.dat");
    ballfinder_  = new BallFinder(calibration_path+"/Param.txt");


    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    img_sub_= it.subscribe("/front_vision/image_raw", 1, &Front_Vision::imageCallback,this);

    ros::NodeHandle node;
    ballinfo_pub_  = node.advertise<nubot_common::FrontBallInfo>("/front_vision/FrontBallInfo",1);

}
~Front_Vision()
{


}

private:


 void
 imageCallback(const sensor_msgs::ImageConstPtr& _image_msg)
 {
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr=cv_bridge::toCvShare(_image_msg,encodings::BGR8);
    Mat image=cv_ptr->image;
    if(image.channels()!=3)
    {
        ROS_WARN("this image is not three channels ");
        return;
    }
    PPoint real_pt;
    colorsegmet_->Segment(image);
    ball_info_.pos_known=ballfinder_->Process(colorsegmet_->segment_result_,real_pt);
    ball_info_.header.stamp = _image_msg->header.stamp;
    ball_info_.header.seq++;
    ball_info_.real_pos.angle  = real_pt.angle_.radian_;
    ball_info_.real_pos.radius = real_pt.radius_;
    ballinfo_pub_.publish(ball_info_);
    DPoint pt(real_pt);
    ROS_INFO("real_pos(%.2f, %.2f)", pt.x_,pt.y_);
}

};


}// end of namespace nubot


int main(int argc, char **argv)
{
     ros::init(argc,argv,"front_vision");
     ros::Time::init();
     ROS_INFO("start front_vision process");
     nubot::Front_Vision front_vision(argc, argv);
     ros::spin();
     return 0;
}
