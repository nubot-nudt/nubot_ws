#ifndef __NUBOT_VISION_WHITES_H_
#define __NUBOT_VISION_WHITES_H_

#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/scanpoints.h"
#include "nubot/omni_vision/transfer.h"

namespace nubot
{ 

using std::vector;
using std::string;

const int MAX_NUMBRT_CONST = 10000;
const int MIN_NUMBRT_CONST = -10000;

class Whites
{
public:

	struct peak
	{
		int peak_index;
		int left_hollow;
		int right_hollow;
		int near_hollow;
		int width;
		bool boundaries;
	};

public:

    Whites(ScanPoints & _scanpts, Transfer & _coor_transfer);
	
	void process();

    void detectWhitePts(std::vector<DPoint2i> & pts,std::vector<uchar> & _ColorY_Aver,std::vector<DPoint2i> & whites);
	
    void detectWave(std::vector<uchar> & colors,std::vector<bool> & wave_hollow,std::vector<bool> & wave_peak);

    void findNearHollow(vector<uchar> & colors,vector<bool> & wave_peak,vector<bool> & wave_hollow,vector<peak> & peak_count);

    bool IsWhitePoint(std::vector<DPoint2i> & pts,double _color_average,vector<uchar> & colors,peak & peak_count );
	
	void calculateWeights();
	
    void showWhitePoints(cv::Mat & _img);

    void showWhitePoints(cv::Mat &  _img,DPoint _robot_loc, Angle _angle, int filed_length =1920,int filed_width=1314);

    ScanPoints   * scanpts_;
    Transfer     * transfer_;

	vector<DPoint2i> img_white_;
  	vector<PPoint>   robot_white_;
    vector<double>   weights_;

private:
	int h_low_;
	int h_high_;
	int nums_pts_line_;
    int filter_width_;          
    int merge_wave_;
	float t_new[256];
};

}
#endif  //!__NUBOT_VISION_WHITES_H_

