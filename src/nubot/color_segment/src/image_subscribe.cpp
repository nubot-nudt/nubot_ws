#include "image_subscribe.h"

image_subscribe::image_subscribe(ros::NodeHandle node):node_(node)
{
    ros::Time::init();
    image_transport::ImageTransport it(node_);
    img_sub_= it.subscribe("/camera/image_raw", 1, &image_subscribe::imageCallback,this);
   // img_sub_= it.subscribe("/front_vision/image_raw", 1, &image_subscribe::imageCallback,this);
    update_img=false;
}
image_subscribe::~image_subscribe()
{

}

void
image_subscribe::imageCallback(const sensor_msgs::ImageConstPtr& _image_msg)
{
   cv_bridge::CvImageConstPtr cv_ptr;
   cv_ptr=cv_bridge::toCvShare(_image_msg,sensor_msgs::image_encodings::BGR8);
   if(cv_ptr->image.channels()<3)
   {
      ROS_WARN("the image doesn't have three channels");
      return ;
    }
   if(update_img)
    {
      receive_img_=cv_ptr->image.clone();
      update_img=false;
    }
}
cv::Mat
image_subscribe::get_camera_img()
{
    return receive_img_.clone();
}

void image_subscribe::set_update_state(bool is_update)
{
    update_img=is_update;
}

bool image_subscribe::get_update_state()
{
    return update_img;
}
void
image_subscribe::run()
{
    ros::spin();
}
