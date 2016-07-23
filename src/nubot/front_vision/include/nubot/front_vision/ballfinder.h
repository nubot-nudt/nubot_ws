#ifndef __NUBOT_VISION_BALL_H_
#define __NUBOT_VISION_BALL_H_

#include "nubot/core/core.hpp"
#include "opencv2/core/core.hpp"
#include <cmath>
#include <vector>
#include <deque>
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
  cv::Point center_;
  int area_size_;
  cv::Rect area_rect_;//ex-rectangle
  std::vector<cv::Point> edge_points_;//the edge points(eight neighbors)
};

class C3DConstruct//目标三维重构相关类封装 yqh
{
public:
    bool InitParam(std::string filename);//指定文件来初始化重构参数: 内参 外参 尺寸等
    void PixelPos2CamNormPos(int PixelPosX, int PixelPosY, double CamNormPos[2]);//像素坐标变换到摄像机齐次坐标
    void CamNormPos2BodyPos(double CamBormPos[2], double BodyPos[2], int FV_BallRadius=110);//摄像机齐次坐标变换到机器人坐标 注意是mm
private:
    double KK[9];//摄像机内参矩阵 由fc alpha和cc计算得到
    double KKinvers[9];//摄像机内参矩阵的逆 由fc alpha和cc计算得到
    double kc[5];//径向和切向畸变系数
    double Rc[9];//摄像机外参旋转矩阵 由机器人到摄像机 单位mm
    double Tc[3];//摄像机外参平移向量 由机器人到摄像机 单位mm
    void MatrixMultiplyVector(double matrixA[9],double vectorIn[3], double vectorOut[3]);//矩阵乘以向量
    bool InversMatrix(double c[9],double d[9]);//三阶矩阵的逆(输入输出不能为同一数组)
};

class BallFinder
{

public:

    ImageArea ball_area_;
    BallFinder(std::string _param_path);
    ~BallFinder();
    bool Process(cv::Mat &_image,PPoint &  _real_pt);
    /*the color of the image should be organized just the "Inverse" of the table. e.g.
     *  if the image is BGR image, the table index must be RGB.*/

    bool RegionSearch(cv::Mat & _segment_img,std::vector<ImageArea> &_target_areas, const int &_max_num_of_areas=1,
                      const cv::Rect &_ROI=cv::Rect(0,0,INT_MAX,INT_MAX), const int &_threshold_size=20,
                      const int &_threshold_combination=1, unsigned char _target_color=VISION_COLORSEGMENT_YELLOW);
    bool RegionSearch(cv::Mat & _segment_img,ImageArea &_target_area);

private:
    cv::Mat check_flag_;  //when you need to do image area process, this mat is used to flag the points temporarily
    std::deque<cv::Point> grow_queue_;//the point queue when region grow
    C3DConstruct ReConstructer;
};



}
#endif  //!__NUBOT_VISION_BALL_H_

