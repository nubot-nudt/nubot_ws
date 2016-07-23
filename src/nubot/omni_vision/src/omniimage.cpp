#include "nubot/omni_vision/omniimage.h"
#include <fstream>
using namespace nubot;

OmniImage::OmniImage(std::string infopath)
{
    cv::FileStorage roi_file(infopath, cv::FileStorage::READ);
    int tmp1,tmp2;
    roi_file["center_point_row"]>>tmp2;
    roi_file["center_point_column"]>>tmp1;
    img_ROI_.center_=DPoint2d(tmp1,tmp2);
    small_ROI_.center_=img_ROI_.center_;
    roi_file["big_radius"] >>img_ROI_.radius_;
    roi_file ["small_radius"] >> small_ROI_.radius_;
    roi_file ["rows"]>>height_;
    roi_file ["cols"]>>width_;
    img_ROI_.radius_  -=3;
   // small_ROI_.radius_+=5;
    roi_file.release();
}
OmniImage::~OmniImage()
{

}

bool 
OmniImage::setBGRImage(const Mat & img){
   if(img.channels()!=3)
	   return false;
   bgr_image_=img.clone();
   return true;
}
bool 
OmniImage::setHSVImage(const Mat & img){
	if(img.channels()!=3)
		return false;
	hsv_image_=img.clone();
	return true;
}

bool 
OmniImage::setYUVImage(const Mat & img){
	if(img.channels()!=3)
		return false;
	yuv_image_=img.clone();
	return true;
}
Mat 
OmniImage::getBGRImage(){
    return bgr_image_;
}
Mat  
OmniImage::getHSVImage(){
    return hsv_image_;
}
Mat  
OmniImage::getYUVImage(){
    return yuv_image_;
}

void
OmniImage::bgr2hsv(){
    cvtColor( bgr_image_, hsv_image_, CV_BGR2HSV);
}
void
OmniImage::bgr2yuv(){
    cvtColor( bgr_image_, yuv_image_, CV_BGR2YUV);
}
void
OmniImage::yuv2bgr(){
    cvtColor( yuv_image_, bgr_image_, CV_YUV2BGR);
}
void
OmniImage::yuv2hsv(){
    cv::cvtColor( yuv_image_, bgr_image_, CV_YUV2BGR);
    bgr2hsv();
}
void
OmniImage::hsv2yuv(){
    cvtColor( yuv_image_, bgr_image_, CV_YUV2BGR);
    bgr2yuv();
}
void
OmniImage::hsv2bgr(){
    cvtColor( bgr_image_, yuv_image_, CV_HSV2BGR);
}


Vec3b
OmniImage::bgr2hsv(Vec3b bgr)
{
     const  int hsv_shift = 12;
     static int sdiv_table[256];
     static int hdiv_table180[256];
     static int hdiv_table256[256];
     static volatile bool initialized = false;
     int hrange=180;//or 256;
     int hr = hrange;
     const int* hdiv_table = hr == 180 ? hdiv_table180 : hdiv_table256;
     if( !initialized )
     {
         sdiv_table[0] = hdiv_table180[0] = hdiv_table256[0] = 0;
         for(int i = 1; i < 256; i++ )
         {
               sdiv_table[i]    = cv::saturate_cast<int>((255 << hsv_shift)/(1.*i));
               hdiv_table180[i] = cv::saturate_cast<int>((180 << hsv_shift)/(6.*i));
               hdiv_table256[i] = cv::saturate_cast<int>((256 << hsv_shift)/(6.*i));
         }
         initialized = true;
     }
    Scalar color_bgr(bgr[0],bgr[1],bgr[2]);

    int b = color_bgr[0], g = color_bgr[1], r = color_bgr[2];
    int h, s, v = b;
    int vmin = b, diff;
    int vr, vg;

    if(v < g)    v=g;
    if(v < r)    v=r;
    if(vmin > g) vmin=g;
    if(vmin > r) vmin=r;
    diff = v - vmin;
    vr = v == r ? -1 : 0;
    vg = v == g ? -1 : 0;

    s = (diff * sdiv_table[v] + (1 << (hsv_shift-1))) >> hsv_shift;
    h = (vr & (g - b)) +
        (~vr & ((vg & (b - r + 2 * diff)) + ((~vg) & (r - g + 4 * diff))));
    h = (h * hdiv_table[diff] + (1 << (hsv_shift-1))) >> hsv_shift;
    h += h < 0 ? hr : 0;

    cv::Vec3b HSI;
    HSI[0] =cv::saturate_cast<uchar>(h);
    HSI[1] = (uchar)s;
    HSI[2] = (uchar)v;
    return HSI;
}

void 
OmniImage::getColorValue(DPoint2i & pts,uchar & color_value,int channels,int flags){
	CV_Assert(channels<3);
	cv::Mat *img;
	switch(flags)
	{
	case SELECTION_YUV :img=&yuv_image_;break;
    case SELECTION_RGB :img=&bgr_image_;break;
	case SELECTION_HSI :img=&hsv_image_;break;
	default:img=&yuv_image_;break;
	}
    if(img->empty())
		return ;
    uchar*	ptr_image  = img->data;
    ptr_image  = img->data+img->step[0]*pts.y_+img->step[1]*pts.x_;
    color_value=ptr_image[0];

}
void 
OmniImage::getColorValues(std::vector<DPoint2i> & pts,std::vector<uchar> & color_value,int channels,int flags){
	CV_Assert(channels<3);
	size_t numsPoints=pts.size();
	if(pts.size()!=color_value.size())
	    color_value.resize(numsPoints);
	Mat *img;
    switch(flags)
    {
    case SELECTION_YUV :img=&yuv_image_;break;
    case SELECTION_RGB :img=&bgr_image_;break;
    case SELECTION_HSI :img=&hsv_image_;break;
    default:img=&yuv_image_;break;
    }
    uchar*	ptr_image  = img->data;
	for (size_t i=0; i<numsPoints; i++)       
	{
        ptr_image  = img->data+img->step[0]*pts[i].y_+img->step[1]*pts[i].x_;
        color_value[i]=ptr_image[0];
    }
}

void
OmniImage::setSmallROI(Circle _circle){
    small_ROI_=_circle;
}
void
OmniImage::setBigROI(Circle _circle){
    img_ROI_=_circle;
}
void
OmniImage::setWidth(int _width){
    width_=_width;
}
void
OmniImage::setHeight(int _height){
    height_=_height;
}
Circle
OmniImage::getSmallROI(){
    return  small_ROI_;
}
Circle
OmniImage::getBigROI(){
    return img_ROI_;
}
int
OmniImage::getWidth(){
    return width_;
}
int
OmniImage::getHeight()
{
    return height_;
}
