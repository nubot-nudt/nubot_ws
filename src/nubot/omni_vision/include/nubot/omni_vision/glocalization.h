#ifndef __NUBOT_VISION_GLOCALIZATION_H_
#define __NUBOT_VISION_GLOCALIZATION_H_


#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/optimise.h"
#include "nubot/omni_vision/fieldinformation.h"

namespace nubot
{ 

class Globallocalization
{
public:
	struct location_err
	{
        DPoint loction;
		Angle  ang;
		double error;
	};

public:
    Globallocalization(Optimise & _optimise);

    void rankErrorsforSamples(const std::vector<double> & _weights,const std::vector<DPoint> &  _white_pt, Angle & _angle,int _times);
	bool process(const std::vector<double> & _weights,const std::vector<PPoint> &  _white_pt,DPoint & _robotloction, Angle & _angle);

	void showWhitePoints(const std::vector<double> & _weights,const std::vector<PPoint> &  _white_pt,DPoint & _robotloction, Angle & _angle);
	
    Optimise * opti_;
    FieldInformation fildinfo_;
	std::vector<location_err> samples_;
	
};


}
#endif  //!__NUBOT_VISION_GLOCALIZATION_H_

