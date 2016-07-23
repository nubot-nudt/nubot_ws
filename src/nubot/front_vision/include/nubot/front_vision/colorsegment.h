#ifndef __NUBOT_VISION_COLORSEGMENT_H_
#define __NUBOT_VISION_COLORSEGMENT_H_


#include <opencv2/opencv.hpp>
#include <string>
namespace nubot
{
  class ColorSegment
  {
  private:
    //int width_;   //The width of the picture, as well as the "Color Segment Result"
	//int height_;  //The height of the picture, as well as the "Color Segment Result"
    unsigned char table_[64*64*64];  // the color table for segmentation
  public:
    cv::Mat segment_result_;//the result of segmentation;
    ColorSegment(std::string _table_name);
    bool Segment(cv::Mat &_image, const cv::Rect &_ROI=cv::Rect(0,0,INT_MAX,INT_MAX));
  };
}

#endif//!__NUBOT_VISION_COLORSEGMENT_H_
