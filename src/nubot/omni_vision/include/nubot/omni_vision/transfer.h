#ifndef __NUBOT_VISION_TRANSFER_H_
#define __NUBOT_VISION_TRANSFER_H_

#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/omniimage.h"


namespace nubot
{ 

using std::vector;
class Transfer
{
public:
	//! read distance calibration  result and image infomation and location info;
    Transfer(std::string infopath,OmniImage & _omni_img);

    //! get the  image coordinates of the object from the robot coordinates
    void calculateImageCoordinates(const vector<PPoint> & real_coor, vector<DPoint2i> & img_pts);

    //! get the  image coordinate of the object from the robot coordinates
    void calculateImageCoordinates(const PPoint & real_coor, DPoint2i & img_pt);

    //! get the  image coordinates of the object from the  global world coordinates
    void calculateImageCoordinates(const vector<DPoint> & world_coor,const DPoint & location ,
                                               const Angle & facing, vector<DPoint2i> & img_pts);

    //! get the  image coordinate of the object from the  global world coordinate
    void calculateImageCoordinates(const DPoint & world_coor,const DPoint & location ,
                                   const Angle & facing, DPoint2i & img_pt);


	//!  get the actual distance between the robot and object location from image coordinate 
    void calculateRealDistance(const DPoint2i & img_coor,double & distance);

    //!  get the actual distances between the robot and object locations from image coordinate
    void  calculateRealDistance(const std::vector<DPoint2i>& img_coor,vector<double> & distances);

	//!  get the robot coordinate of object from image coordinate
    void calculateRealCoordinates(const DPoint2i & img_coor , PPoint & real_pt,bool _correction=false);

    //!  get the robot coordinates of object from image coordinates
    void calculateRealCoordinates(const std::vector<DPoint2i>& img_coor, vector<PPoint> & real_pts,bool _correction=false);

    //!  get the robot coordinate of object from global world coordinate
    void calculateRealCoordinates(const DPoint & world_coor,const DPoint & location ,
                                  const Angle & facing,PPoint & real_pt,bool _correction=false);

    //!  get the robot coordinates of object from global world coordinates
    void calculateRealCoordinates(const std::vector<DPoint>& world_coor,const DPoint & location ,
                                  const Angle & facing,vector<PPoint> & real_pts,bool _correction=false);

    //!  robot coor rotate angle
    void rotate(const vector<PPoint>& robot_coor,const Angle & ang,vector<PPoint> & real_pts);

	//! get the global world coordinates of object from robot coordinate
    void calculateWorldCoordinates(const PPoint & polar_coor,const DPoint & location ,
                                   const Angle & facing, DPoint & world_pt);

    //! get the global world coordinates of object from robot coordinates
    void calculateWorldCoordinates(const vector<PPoint> & polar_coor,const DPoint & location ,
                                   const Angle & facing,vector<DPoint> & world_pts);

    //! get the  global world  coordinates of object from image coordinate
    void calculateWorldCoordinates(const DPoint2i & img_coor,const DPoint & location ,
                                   const Angle & facing, DPoint & world_pt);

    //! get the  global world coordinates of object from image coordinates
    void calculateWorldCoordinates(const vector<DPoint2i> & img_coor,const DPoint & location ,
                                   const Angle & facing, vector<DPoint> & world_pts);

    OmniImage * omni_img_; //<-image information

    void correct_offset(DPoint & _loc);
    DPoint get_offset();

private:
    /*! @brief the center is got by calibration the mirror*/
    DPoint center_;
    int height_;
    int width_;
    double para_a_;
    double para_b_;
    DPoint off_location_;
};

}

#endif   //__NUBOT_VISION_TRANSFER_H_
