#ifndef IMAGE_SUBSCRIBE_H
#define IMAGE_SUBSCRIBE_H

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

#include <QtCore>

class image_subscribe: public QThread
{
public:
    image_subscribe(ros::NodeHandle);
    image_subscribe();
    void imageCallback(const sensor_msgs::ImageConstPtr& _image_msg);
    void run();
      ~image_subscribe();
public:

    cv::Mat receive_img_;
    cv::Mat get_camera_img();
    void set_update_state(bool is_update);
    bool get_update_state();
    bool update_img;
    image_transport::Subscriber img_sub_;
    ros::NodeHandle node_;
};

#endif // IMAGE_SUBSCRIBE_H
