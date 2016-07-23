#include <fstream>
#include "nubot/omni_vision/colorsegment.h"
#include <iostream>
#include "nubot/core/core.hpp"
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
  uchar* ptr_result = segment_result_.data;
  uchar* ptr_image = _image.data;
  int i(0),j(0),k(0),index(0);
#ifdef using_openmp
#pragma omp parallel for private(index,k,i,j)
#endif
  for(i=0; i<_image.rows; i++)
  {
      for(j=0; j<_image.cols; j++)
      {
          index = segment_result_.step[0]*i+ segment_result_.step[1] *j;
          k =  _image.step[0]*i+_image.step[1]*j;
          if((Ymin<=i)&&(i<=Ymax)&&(Xmin<=j)&&(j<=Xmax))
              ptr_result[index] = table_[ptr_image[k]/4*64*64 + ptr_image[k+1]/4*64 + ptr_image[k+2]/4];
      }
  }
  return(true);
}
