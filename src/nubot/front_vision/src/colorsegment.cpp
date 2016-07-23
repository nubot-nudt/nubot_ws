#include <fstream>
#include <iostream>
#include "nubot/front_vision/colorsegment.h"

nubot::ColorSegment::ColorSegment(std::string _table_name)
{
    std::ifstream CTable_Read(_table_name.c_str(), std::ios::binary|std::ios::in);
    CTable_Read.read((char *)table_,sizeof(unsigned char)*64*64*64);
    CTable_Read.close();
}

bool
nubot::ColorSegment::Segment(cv::Mat &_image, const cv::Rect &_ROI)
{
  if(_image.cols==0 || _image.rows==0)
    return(false);
  else if(_image.cols!=segment_result_.cols || _image.rows!=segment_result_.rows)
    segment_result_.create(_image.rows,_image.cols,CV_8UC1);
  if(_image.channels()<3)
    return(false);
  int Xmin = _ROI.x;
  int Ymin = _ROI.y;
  int Xmax = _ROI.x+_ROI.width;
  int Ymax = _ROI.y+_ROI.height;
  uchar*	ptr_result = segment_result_.data;
  uchar*	ptr_image = _image.data;
  for(int i=0; i<_image.rows; i++)
  {
    ptr_result = segment_result_.data+segment_result_.step[0]*i;
    ptr_image  = _image.data+_image.step[0]*i;
    for(int j=0; j<_image.cols; j++)
    {
      if((Ymin<=i)&&(i<=Ymax)&&(Xmin<=j)&&(j<=Xmax))
        *ptr_result = table_[ptr_image[0]/4*64*64 + ptr_image[1]/4*64 + ptr_image[2]/4];
      ptr_result += segment_result_.step[1];
      ptr_image  += _image.step[1];
    }
  }
  return(true);
}
