#ifndef __NUBOT_VISION_ODOMETRY_H_
#define __NUBOT_VISION_ODOMETRY_H_


#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
 

namespace nubot
{ 

const int CODER_LINE_NUMS_CONST  =  60*1000/26;//500*4;
const int GEAR_RATIO_CONST       =	14   ;
const int GEAR_CLEARANCE_CONST	 =  100	 ;		 
const int WHEELE_DIA_CONST       =  120  ;
const int WHEELE_DISTANCE_CONST  =  370  ;
const int MOTOR_NUMS_CONST       =  3  ;
const double CIRCLE_COUNT_CONST  =  1000/26;

class Odometry
{
public:
    Odometry(void)   ;
    bool process(std::vector<double> & _motor_data,double duration);
    void clear(const Angle & _angle);
    
    double getAngularVelocity();
    DPoint getWorldVelocity();
    DPoint getRealVelocity();

    Angle  getDeltaAngle();
    DPoint getRealLocaton();
    DPoint getWorldLocaton();

private:
    /*! @brief the robot's orientation angle */
    Angle  angle_;
    Angle  delta_angle_;
    DPoint delta_real_pos_;
    DPoint delta_world_pos_;

    DPoint world_velocity_;
    DPoint real_velocity_;
    double angular_velocity_;
};

}
#endif  //!__NUBOT_VISION_ODOMETRY_H_

