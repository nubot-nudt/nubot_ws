#ifndef __NUBOT_VISION_WHITES_H_
#define __NUBOT_VISION_WHITES_H_

#include <opencv2/opencv.hpp>
#include "core/core.hpp"
#include "scanpoints.h"

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

    Whites(ScanPoints & _scanpts);
	
	void process();

    void detectWhitePts(std::vector<DPoint2i> & pts,std::vector<uchar> & _ColorY_Aver);
	
    void detectWave(std::vector<uchar> & colors,std::vector<bool> & wave_hollow,std::vector<bool> & wave_peak);

    void findNearHollow(vector<uchar> & colors,vector<bool> & wave_peak,vector<bool> & wave_hollow,vector<peak> & peak_count);

    bool IsWhitePoint(std::vector<DPoint2i> & pts,double _color_average,vector<uchar> & colors,peak & peak_count);

    void showWhitePoints();

    ScanPoints   * scanpts_;

    vector<DPoint2i> img_white_;
    vector<PPoint>   robot_white_;
    vector<double>   weights_;

    cv::Mat image_show_;
    std::vector<nubot::DPoint2i> image_whites_;
    static void  on_mouse(int event, int x, int y, int flags, void* tmpThis);
    bool pnpoly (const std::vector<DPoint2i> & pts, DPoint2i test_pt);
private:
    double factor_;
	int h_low_;
	int h_high_;
	int nums_pts_line_;
    int filter_width_;          
    int merge_wave_;
    std::vector<float> t_new;
};

}
#endif  //!__NUBOT_VISION_WHITES_H_

