#ifndef __NUBOT_VISION_OMNIIMAGE_H_
#define __NUBOT_VISION_OMNIIMAGE_H_


#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"

namespace nubot
{ 

using std::vector;
using cv::Mat;
using cv::Scalar;
using cv::Vec3b;

enum 
{	
	SELECTION_RGB=1,
	SELECTION_YUV=2,
	SELECTION_HSI=3,
};

class OmniImage
{

public:
	
    OmniImage(std::string infopath);
    ~OmniImage();
    bool setBGRImage(const Mat & img);
    bool setHSVImage(const Mat & img);
    bool setYUVImage(const Mat & img);

    Mat  getBGRImage();
    Mat  getHSVImage();
    Mat  getYUVImage();

    void bgr2hsv();
    void bgr2yuv();
    void yuv2bgr();
	void yuv2hsv();
	void hsv2yuv();
    void hsv2bgr();

    /*! get the code from opencv*/
    cv::Vec3b bgr2hsv(cv::Vec3b bgr);

    void getColorValue(DPoint2i & pts,uchar & color_value,int channels,int flags);
    void getColorValues(std::vector<DPoint2i> & pts,std::vector<uchar> & color_value,int channels,int flags);

    void setSmallROI(Circle _circle);
    void setBigROI(Circle _circle);
    void setWidth(int _width);
    void setHeight(int _height);

    Circle getSmallROI();
    Circle getBigROI();
    int getWidth();
    int getHeight();

public:
    /*! @brief the bgr image.*/
    Mat bgr_image_;
    /*! @brief the hsv image.*/
	Mat hsv_image_; 
    /*! @brief the yuv image.*/
	Mat yuv_image_; 

private:
    /*! @brief the big ROI area which should be processed.*/
	Circle img_ROI_;
    /*! @brief the small ROI area which should be the robot itself*/
	Circle small_ROI_;
    /*! @brief the width(cols) of the image*/
    int width_;
    /*! @brief the height(rows) of the image.*/
	int height_;
};

}

#endif  //!__NUBOT_VISION_OMNIIMAGE_H_

