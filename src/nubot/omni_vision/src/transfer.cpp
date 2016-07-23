#include "nubot/omni_vision/transfer.h"
#include <fstream>

using namespace nubot;

Transfer::Transfer(std::string infopath,OmniImage & _omni_img)
{
    omni_img_=&_omni_img;
    width_ =omni_img_->getWidth();
    height_=omni_img_->getHeight();

    cv::FileStorage calibration(infopath, cv::FileStorage::READ);
    double temp1,temp2;
    calibration["center_coloum"]>>temp1;
    calibration["center_row"]>>temp2;
    center_=DPoint(temp1,temp2);

    calibration["para_a"]>>para_a_;
    calibration["para_b"]>>para_b_;

    calibration["correction_x"]>>temp1;
    calibration["correction_y"]>>temp2;
    off_location_=DPoint(temp1,temp2);

    calibration.release();
}

void
Transfer::calculateImageCoordinates(const std::vector<PPoint> & real_coor, vector<DPoint2i> & img_pts)
{
    img_pts.clear();
    size_t numtrans=real_coor.size();
    img_pts.resize(numtrans);
#ifdef using_openmp
#pragma omp parallel for
#endif
    for(size_t i=0; i<numtrans; i++)
       calculateImageCoordinates(real_coor[i],img_pts[i]);
}

void
Transfer::calculateImageCoordinates(const PPoint & real_coor, DPoint2i & img_pt)
{
    double image_distance=std::atan(real_coor.radius_/(100.0*para_a_))/para_b_;
    DPoint2i image_pt=DPoint2i(int(image_distance*cos(real_coor.angle_.radian_)),
                               int(image_distance*sin(real_coor.angle_.radian_)));
    img_pt = image_pt+DPoint2i(center_);
}

void
Transfer::calculateImageCoordinates(const vector<DPoint> & world_coor,const DPoint & location ,
                                         const Angle & facing, std::vector<DPoint2i> & img_pts)
{
    vector<PPoint> real_pts;
    calculateRealCoordinates(world_coor,location,facing,real_pts);
    calculateImageCoordinates(real_pts,img_pts);

}

void
Transfer::calculateImageCoordinates(const DPoint & world_coor,const DPoint & location ,
                                             const Angle & facing ,DPoint2i & img_pt)
{
   PPoint real_pts;
   calculateRealCoordinates(world_coor,location,facing,real_pts);
   calculateImageCoordinates(real_pts,img_pt);
}


void
Transfer::calculateRealDistance(const DPoint2i & img_coor, double & distance)
{
    DPoint2d temptrans=DPoint2d(img_coor)-center_;
    double pixel_dis=temptrans.norm();
    distance = para_a_*tan(para_b_*pixel_dis)*100;
}

void
Transfer::calculateRealDistance(const std::vector<DPoint2i>& img_coor, std::vector<double> & distances)
{
	size_t numtrans=img_coor.size();
    distances.clear();
    distances.resize(numtrans);
#ifdef using_openmp
#pragma omp parallel for
#endif
	for(size_t i=0; i<numtrans; i++)
       calculateRealDistance(img_coor[i],distances[i]);
}


//!  get the real coordinate of object from image coordinate
void
Transfer::calculateRealCoordinates(const DPoint2i & img_coor,PPoint &  real_pt,bool _correction)
{
    DPoint2d temptrans=DPoint2d(img_coor)-center_;
    double distance ;
    calculateRealDistance(img_coor,distance);
    real_pt = PPoint(temptrans.angle(),distance);
    if(_correction)
    {
        DPoint pt = DPoint(real_pt);
        correct_offset(pt);
        real_pt = PPoint(pt);
    }
}

void
Transfer::calculateRealCoordinates(const std::vector<DPoint2i>& img_coor,std::vector<PPoint> & real_pts,bool _correction)
{
	size_t numtrans=img_coor.size();
    real_pts.clear();
    real_pts.resize(numtrans);
#ifdef using_openmp
#pragma omp parallel for
#endif
	for(size_t i=0; i<numtrans; i++)
       calculateRealCoordinates(img_coor[i],real_pts[i],_correction);
}
//!  get the robot coordinate of object from global world coordinate
void
Transfer::calculateRealCoordinates(const DPoint & world_coor,const DPoint & location ,
                                   const Angle & facing,PPoint &  real_pt,bool _correction)
{
	DPoint pt=world_coor-location;
	PPoint pts(pt);
    real_pt = PPoint(pts.angle_-facing,pts.radius_);
    if(_correction)
    {
        DPoint pt = DPoint(real_pt);
        correct_offset(pt);
        real_pt = PPoint(pt);
    }
}

//!  get the robot coordinates of object from global world coordinates
void
Transfer::calculateRealCoordinates(const std::vector<DPoint>& world_coor,const DPoint & location ,
                                   const Angle & facing,vector<PPoint> & real_pts,bool _correction)
{
	size_t numtrans=world_coor.size();
    real_pts.clear();
    real_pts.resize(numtrans);
#ifdef using_openmp
#pragma omp parallel for
#endif
	for(size_t i=0; i<numtrans; i++)
         calculateRealCoordinates(world_coor[i],location,facing,real_pts[i],_correction);
}

void
Transfer::calculateWorldCoordinates(const PPoint & polar_coor,const DPoint & location ,
                                    const Angle & facing, DPoint & world_pt)
{

    PPoint pt(polar_coor.angle_+facing , polar_coor.radius_);
    world_pt = location + DPoint(pt);
}

void
Transfer::calculateWorldCoordinates(const vector<PPoint> & polar_coor,const DPoint & location ,
                                    const Angle & facing,std::vector<DPoint> & world_pts)
{
	size_t numtrans=polar_coor.size();
    world_pts.clear();
    world_pts.resize(numtrans);
#ifdef using_openmp
#pragma omp parallel for
#endif
	for(size_t i=0; i< numtrans; i++)
         calculateWorldCoordinates(polar_coor[i], location,  facing, world_pts[i]);
}

void
Transfer::calculateWorldCoordinates(const DPoint2i & img_coor,const DPoint & location ,
                                    const Angle & facing,DPoint & world_pt)
{
    PPoint pt;
    calculateRealCoordinates(img_coor , pt);
    calculateWorldCoordinates(pt, location, facing, world_pt);
}

void
Transfer::calculateWorldCoordinates(const vector<DPoint2i> & img_coor,const DPoint & location ,
                                    const Angle & facing,std::vector<DPoint> & world_pts)
{
	
	size_t numtrans=img_coor.size();
    world_pts.clear();
    world_pts.resize(numtrans);
#ifdef using_openmp
#pragma omp parallel for
#endif
	for(size_t i=0; i< numtrans; i++)
       calculateWorldCoordinates(img_coor[i], location, facing, world_pts[i]);
}

void
Transfer::rotate(const vector<PPoint>& robot_coor,const Angle & ang,vector<PPoint> & real_pts)
{
	size_t numtrans=robot_coor.size();
    real_pts.clear();
    real_pts.resize(numtrans);
#ifdef using_openmp
#pragma omp parallel for
#endif
	for(size_t i=0; i< numtrans; i++)
         real_pts[i]=PPoint(robot_coor[i].angle_+ang,robot_coor[i].radius_);
}

void
Transfer::correct_offset(DPoint & _loc)
{
    _loc=_loc-off_location_;
}

DPoint
Transfer::get_offset()
{
    return off_location_;
}
