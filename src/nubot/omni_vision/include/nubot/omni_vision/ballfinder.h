#ifndef __NUBOT_VISION_BALL_H_
#define __NUBOT_VISION_BALL_H_

#include "nubot/omni_vision/transfer.h"
#include "nubot/omni_vision/omniimage.h"
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/fieldinformation.h"

//Macro definition for color segmentation
#define VISION_COLORSEGMENT_YELLOW	0	//Yellow represent for football
#define VISION_COLORSEGMENT_BLACK	1	//green represent for ground
#define VISION_COLORSEGMENT_GREEN	2	//green represent for ground
#define VISION_COLORSEGMENT_UNKNOWCOLOR	3	//Unknown color

namespace nubot
{ 
const int BALL_PREDICT_BIAS_CONST=60;

struct ImageArea
{
  cv::Point area_center_;
  int area_size_;
  cv::Rect area_rect_;//ex-rectangle
  std::vector<cv::Point> edge_points_;//the edge points(eight neighbors)
};

class BallFinder
{



public:

    ImageArea ball_area_;
    BallFinder(Transfer & _transfer);
    ~BallFinder();
    bool Process(cv::Mat &_image,const DPoint & _location, const Angle & _anlge);
    DPoint get_ball_global_loc();
    PPoint get_ball_real_loc();

    /*the color of the image should be organized just the "Inverse" of the table. e.g.
     *  if the image is BGR image, the table index must be RGB.*/

    bool RegionSearch(cv::Mat & _segment_img,std::vector<ImageArea> &_target_areas, const int &_max_num_of_areas=1,
                      const cv::Rect &_ROI=cv::Rect(0,0,INT_MAX,INT_MAX), const int &_threshold_size=15,
                      const int &_threshold_combination=1, unsigned char _target_color=VISION_COLORSEGMENT_YELLOW);

    void showBall(cv::Mat & _img);
    void showBall(cv::Mat &  _img,DPoint _robot_loc, Angle _angle, int filed_length =1920,int filed_width=1314);


private:
    Transfer *  transfer_;
    cv::Mat check_flag_;  //when you need to do image area process, this mat is used to flag the points temporarily

    std::vector<DPoint> ball_record_;
    std::vector<double> ball_time_;
    std::vector<DPoint> record_velocity_;

    DPoint ball_global_loc_;
    PPoint ball_real_loc_;
    bool   is_detected_ball_;

    std::deque<cv::Point> grow_queue_;//the point queue when region grow

    FieldInformation field_info_;

};



}
#endif  //!__NUBOT_VISION_BALL_H_

