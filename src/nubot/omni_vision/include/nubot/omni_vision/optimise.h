#ifndef __NUBOT_VISION_OPTIMISE_H_
#define __NUBOT_VISION_OPTIMISE_H_

#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/fieldinformation.h"
#include "nubot/omni_vision/transfer.h"

namespace nubot
{ 

class Optimise
{

public:
	//! read error table and diff table
    Optimise(std::string errorpath,std::string diffpathX,std::string diffpathY,Transfer & _transfercoor);

    double process(const vector<double>& weights,const vector<DPoint> & polar_pts,DPoint & pt , Angle & phi);
    double analyse(const vector<double>& weights,const vector<DPoint> & polar_pts,DPoint2d & hxy, double & hphi,const DPoint & xy, const Angle & phi);
    void   calculateGradient(const vector<double>& weights,const vector<DPoint> & polar_pts,
                            double& err, DPoint2d & grad, double& dphi, const DPoint & xy, const Angle & phi);
	//! the weights of white points 
    double caculateErrors(const vector<double>& weights,const vector<DPoint> & polar_pts,const DPoint & pt,const Angle & phi);

    FieldInformation fildinfo_;
    Transfer * transfer_;
	int startx_ ;
	int endx_   ;
	int starty_ ;
	int endy_   ;
	int xlong_  ; 
	int ylong_  ;

private:
    double *DistoMarkLine;
	double *Diff_X;
	double *Diff_Y;
	cv::Mat errortable_;
	cv::Mat difftable_;
};

}
#endif  //__NUBOT_VISION_OPTIMISE_H_
