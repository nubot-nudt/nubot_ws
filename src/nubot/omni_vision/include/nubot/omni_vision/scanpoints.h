#ifndef __NUBOT_VISION_SCANPOINTS_H_
#define __NUBOT_VISION_SCANPOINTS_H_


#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "nubot/core/core.hpp"
#include "nubot/omni_vision/omniimage.h"


namespace nubot
{ 
using std::vector;

class ScanPoints
{
public:
    ScanPoints(OmniImage & _omni_img);
	vector<DPoint2i>  scanPolarLine(const Angle & ang);
	vector<DPoint2i>  scanVertiLine(const DPoint2i & startPt,const DPoint2i & endPt);
	vector<DPoint2i>  scanHorizLine(const DPoint2i & startPt,const DPoint2i & endPt);
	vector<DPoint2i>  scanLine(const DPoint2i & startPt,const DPoint2i & endPt);

	void showScanPoints();
	
	void process();
    void filter();
    OmniImage * omni_img_;           //<-image information

	vector< vector<DPoint2i> > polar_pts_;
    vector< vector<DPoint2i> > verti_pts_;
	vector< vector<DPoint2i> > horiz_pts_;
    vector< vector<uchar> >    polar_pts_y_;
    vector< vector<uchar> >    verti_pts_y_;
    vector< vector<uchar> >    horiz_pts_y_;


    vector< vector<uchar> >    filter_polar_pts_y_;
    vector< vector<uchar> >    filter_verti_pts_y_;
    vector< vector<uchar> >    filter_horiz_pts_y_;


private:
    int    filter_width_;
    int    interval_;
	double ratio_;
};



}
#endif  //!__NUBOT_VISION_SCANPOINTS_H_

