#ifndef __NUBOT_VISION_FIELDINFOMATION_H_
#define __NUBOT_VISION_FIELDINFOMATION_H_

#include <opencv2/opencv.hpp>
#include "core/core.hpp"

namespace nubot
{ 


using std::vector;
using std::string;

class LineSegment
{
public:

    LineSegment()
    {
      vector_=end_=start_=DPoint(0,0);
    }

    LineSegment(const DPoint & _start, const DPoint & _end)
    {
       start_ =_start;
       end_   =_end;
       vector_=end_-start_;
    }

    LineSegment(const LineSegment & _lineSement)
    {
        start_ =_lineSement.start_;
        end_   =_lineSement.end_;
        vector_=_lineSement.vector_;
    }

    // the distacne between the point and the linesegment.
    double distance(const DPoint & pt) const
    {
        DPoint vector_vp = pt- start_;
        double dotProduct = vector_vp.x_*vector_.x_ + vector_vp.y_ *vector_.y_;
        if(dotProduct <= 0)
            return  pt.distance(start_);
        double LineSegLength = vector_.norm() * vector_.norm();
        if (dotProduct >= LineSegLength)
            return  pt.distance(end_);
        double r = dotProduct / LineSegLength;
        DPoint vert_pt = start_ + r *vector_;
        return  pt.distance(vert_pt);
    }

public:
    DPoint start_;
    DPoint end_;
    DPoint vector_;
};

class FieldInformation
{
public:

    FieldInformation();
    FieldInformation(string infopath);

    bool isInInterRect(DPoint world_pt,double shrink=0);
    bool isInOuterRect(DPoint world_pt,double shrink=0);
	bool isOppfield(DPoint world_pt);
	bool isOurfield(DPoint world_pt);

    std::vector<int> xline_;
    std::vector<int> yline_;
	DPoint opp_goal_[2];
	DPoint our_goal_[2];
	Circle centercircle_;
    std::vector<Circle > postcircle_;

    std::vector<LineSegment> x_white_line_;
    std::vector<LineSegment> y_white_line_;
};



}
#endif  //!__NUBOT_VISION_FIELDINFOMATION_H_

