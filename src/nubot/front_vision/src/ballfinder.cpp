#include "nubot/front_vision/ballfinder.h"
#include <fstream>
#include <iostream>
using namespace nubot;

BallFinder::BallFinder(std::string _param_path)
{
    ReConstructer.InitParam(_param_path);
}

BallFinder::~BallFinder()
{

}

bool
BallFinder::Process(cv::Mat &_image,PPoint & _real_pt)
{
  bool is_detected_ball=false;
  _real_pt=PPoint(Angle(0),0);
  if(_image.cols==0 || _image.rows==0)
     return false;
  if(_image.cols!=check_flag_.cols || _image.rows!=check_flag_.rows)
       check_flag_.create(_image.rows,_image.cols,CV_8UC1);

  is_detected_ball=RegionSearch(_image,ball_area_);

  if(            ball_area_.center_.x <(_image.rows-ball_area_.center_.y)*_image.cols/4/_image.rows
  ||(_image.cols-ball_area_.center_.x)<(_image.rows-ball_area_.center_.y)*_image.cols/4/_image.rows
  || ball_area_.center_.x<_image.cols/10
  || ball_area_.center_.x>_image.cols*9/10)
      is_detected_ball=false;

  if(is_detected_ball)
  {
      double BallCamPos[2];
      double BallBodyPos[2];
      ReConstructer.PixelPos2CamNormPos(ball_area_.center_.x,
                                        ball_area_.center_.y,BallCamPos);
      ReConstructer.CamNormPos2BodyPos(BallCamPos,BallBodyPos);
      DPoint pt(BallBodyPos[0]/10,BallBodyPos[1]/10);
      _real_pt  = PPoint(pt);
  }
  return is_detected_ball;
}



bool
nubot::BallFinder::RegionSearch(cv::Mat & _segment_img,std::vector<ImageArea> &_target_areas, const int &_max_num_of_areas,
                                const cv::Rect &_ROI, const int &_threshold_size, const int &_threshold_combination,
                                unsigned char _target_color)
{
  if(check_flag_.cols!=_segment_img.cols || check_flag_.rows!=_segment_img.rows)
    return(false);
  if(_segment_img.rows*_segment_img.cols<_threshold_size)
    return(false);
  if((double)(_ROI.width)*_ROI.height<_threshold_size)
    return(false);

  //some initializations
  static const cv::Point2i neighbors[8]={cv::Point(-1,-1),cv::Point(0,-1),cv::Point(1,-1),
                                         cv::Point(-1,0),                 cv::Point(1, 0),
                                         cv::Point(-1, 1),cv::Point(0, 1),cv::Point(1, 1)};
  ImageArea current_area;//the current area
  long x_sum, y_sum;//used to calculate the centroid of current area
  int area_rect_Xmin;//the ex-rectangle of current area
  int area_rect_Ymin;//the ex-rectangle of current area
  int area_rect_Xmax;//the ex-rectangle of current area
  int area_rect_Ymax;//the ex-rectangle of current area
  cv::Point current_point;//the current point when processing
  cv::Point neighbor_point;//the neighbor of current point when region grow
  int neighbor_amount;//count the neighbors of current point. if less than 8, current point is edge point

  check_flag_.setTo(0);
  int Xmin = _ROI.x;//can be equal
  int Ymin = _ROI.y;//can be equal
  int Xmax = _ROI.x+_ROI.width;//can NOT be equal
  int Ymax = _ROI.y+_ROI.height;//can NOT be equal
  Xmin = cv::max(0,cv::min(_segment_img.cols,Xmin));
  Xmax = cv::max(0,cv::min(_segment_img.cols,Xmax));
  Ymin = cv::max(0,cv::min(_segment_img.rows,Ymin));
  Ymax = cv::max(0,cv::min(_segment_img.rows,Ymax));
  std::vector<ImageArea> area_list;//once a candidate area is found, it will be pushed in this list

  //start process
  //1. search areas no mater how small they are
  for(int y=Ymin; y<Ymax; y++)
  {
    for(int x=Xmin; x<Xmax; x++)
    {
      current_point = cv::Point(x,y);
      //for every pixel, if it is _target_color, it must be put in an area. the check_flag_ mat marked if a pixel has already been put in a area.
      if(_segment_img.at<unsigned char>(current_point)==_target_color && !check_flag_.at<unsigned char>(current_point))
      {
        //if a pixel is _target_color, but has not been put in an area, a new area is established to take this pixel in.  then find all pixels which is connected with this area and put them in.
        check_flag_.at<unsigned char>(current_point) = true;
        grow_queue_.push_back(current_point);
        x_sum = 0;
        y_sum = 0;
        current_area.edge_points_.clear();
        current_area.area_size_ = 0;
        area_rect_Xmin = area_rect_Xmax = current_point.x;
        area_rect_Ymin = area_rect_Ymax = current_point.y;
        //start to search pixels which is connected with this area and put them in
        while(grow_queue_.size()>0)
        {
          current_point = grow_queue_.front();
          x_sum += current_point.x;
          y_sum += current_point.y;
          current_area.area_size_++;
          neighbor_amount = 0;
          for(int k=0; k<8; k++)
          {
            neighbor_point = current_point+neighbors[k];
            if(neighbor_point.x<Xmax && neighbor_point.x>=Xmin && neighbor_point.y<Ymax && neighbor_point.y>Ymin && _segment_img.at<unsigned char>(neighbor_point)==_target_color)
            {
              neighbor_amount++;
              if(check_flag_.at<unsigned char>(neighbor_point)==false)
              {
                check_flag_.at<unsigned char>(neighbor_point) = true;
                grow_queue_.push_back(neighbor_point);
              }
            }
          }
          if(neighbor_amount<8)
          {
            //storage edge points of current area
            current_area.edge_points_.push_back(current_point);
            //update area rect of current area
            if(current_point.x<area_rect_Xmin)
              area_rect_Xmin = current_point.x;
            else if(current_point.x>area_rect_Xmax)
              area_rect_Xmax = current_point.x;
            if(current_point.y<area_rect_Ymin)
              area_rect_Ymin = current_point.y;
            else if(current_point.y>area_rect_Ymax)
              area_rect_Ymax = current_point.y;
          }
          grow_queue_.pop_front();
        }
        //one area is complete
        if(current_area.area_size_>=_threshold_size)
        {
          current_area.center_ = cv::Point(x_sum/current_area.area_size_,y_sum/current_area.area_size_);
          current_area.area_rect_ = cv::Rect(area_rect_Xmin,area_rect_Ymin,area_rect_Xmax-area_rect_Xmin,area_rect_Ymax-area_rect_Ymin);
          area_list.push_back(current_area);
        }

      }//end of processing one area
    }//for(int x=Xmin; x<Xmax; x++)
  }//for(int y=Ymin; y<Ymax; y++)

  if(area_list.size()==0)
    return(false);

  //2. combine areas nearby
  if(_threshold_combination>0)
  {

  }

  //3. rank the areas by size
  std::vector<ImageArea>::iterator temp_area_iterator;
  std::vector<ImageArea>::iterator current_area_iterator;
  _target_areas.clear();
  for(int i=0; i<_max_num_of_areas; i++)
  {
    if(area_list.size()==0)
      break;
    //search the max size area in area_list and put it in _target_areas.
    current_area_iterator = area_list.begin();
    for(temp_area_iterator=area_list.begin(); temp_area_iterator!=area_list.end(); temp_area_iterator++)
    {
      if(temp_area_iterator->area_size_>current_area_iterator->area_size_)
        current_area_iterator = temp_area_iterator;
    }
    _target_areas.push_back(*current_area_iterator);
    area_list.erase(current_area_iterator);
  }
  return(true);
}

bool
nubot::BallFinder::RegionSearch(cv::Mat & _segment_img,ImageArea &_target_area)
{
  std::vector<ImageArea> areas;
  if(!RegionSearch(_segment_img,areas))
    return(false);
  _target_area = *areas.begin();
  return(true);
}
void C3DConstruct::MatrixMultiplyVector(double matrixA[9],double vectorIn[3], double vectorOut[3])
{
    vectorOut[0]=matrixA[0]*vectorIn[0]+ matrixA[1]*vectorIn[1]+ matrixA[2]*vectorIn[2];
    vectorOut[1]=matrixA[3]*vectorIn[0]+ matrixA[4]*vectorIn[1]+ matrixA[5]*vectorIn[2];
    vectorOut[2]=matrixA[6]*vectorIn[0]+ matrixA[7]*vectorIn[1]+ matrixA[8]*vectorIn[2];
}
bool C3DConstruct::InversMatrix(double c[9],double d[9])//三阶矩阵的逆(输入输出不能为同一数组)
{
    double cc=0;//c行列式的|c|
    double ca[9];//c的伴随矩阵
    double cb[4];//代数余子阵
    int m=0;
    int k=0;
    int l=0;
    int num=-1;

    bool isget=false;

    for (int i=0;i<9;i++)	ca[i]=0;
    for(int i=0;i<4;i++)		cb[i]=0;

    cc=c[0]*c[4]*c[8]+c[3]*c[7]*c[2]+c[1]*c[5]*c[6]-c[2]*c[4]*c[6]-c[0]*c[5]*c[7]-c[1]*c[3]*c[8];
    if (cc==0)return false;//行列式为零

    for(int i=0;i<3;i++)
    {	for (int j=0;j<3;j++)
    {	num=-1;
    for (m=0;m<4;m++)//获取余子式
    {	isget=false;
                //获取余子式每个元素的值
                for (k=0;k<3;k++)
                {	for (l=0;l<3;l++)
                {	if (!isget)
                if ((k!=i)&&(l!=j))
                    if (num<(k*3+l))
                    {
                        num=k*3+l;
                        cb[m]=c[k*3+l];
                        isget=true;
                }	}	}
    }
    ca[j*3+i]=cb[0]*cb[3]-cb[1]*cb[2];
    for (int z=0;z<4;z++)	cb[z]=0;
    if ((i+j)%2!=0)			ca[j*3+i]=0.0-ca[j*3+i];
    d[j*3+i]=ca[j*3+i]/cc;
    }
    }
    return true;
}
bool C3DConstruct::InitParam(std::string filename)
{
#define FV_CalibBoardX	90	//标定板原点与机器人中心之间左右距离mm
#define FV_CalibBoardY	500	//标定板原点与机器人中心之间前后距离mm
    std::ifstream ParamFile(filename.c_str());
    if(!ParamFile.is_open())return false;
    double fc[2], alpha, cc[2];
    ParamFile>>fc[0]>>fc[1];
    ParamFile>>alpha;
    ParamFile>>cc[0]>>cc[1];
    KK[0] = fc[0];		KK[1] = alpha*fc[0];	KK[2] = cc[0];
    KK[3] = 0;			KK[4] = fc[1];			KK[5] = cc[1];
    KK[6] = 0;			KK[7] = 0;				KK[8] = 1;
    if(!InversMatrix(KK,KKinvers))return false;

    double RcTemp[9];//摄像机到标定板转换矩阵 即标定得到的外参 用来进一步得到摄像机到机器人坐标的转换矩阵
    double TcTemp[3];//摄像机到标定板转换矩阵 即标定得到的外参 用来进一步得到摄像机到机器人坐标的转换矩阵
    ParamFile>>RcTemp[0]>>RcTemp[1]>>RcTemp[2]>>TcTemp[0]
            >>RcTemp[3]>>RcTemp[4]>>RcTemp[5]>>TcTemp[1]
            >>RcTemp[6]>>RcTemp[7]>>RcTemp[8]>>TcTemp[2];
    //机器人坐标系为前左上 标定板坐标系为右前上, 机器人在标定板X轴正向 Y轴负向
    //				┌ 0 -1 0	┐				  ┌ FV_CalibBoardX	┐
    //	Rc = RcTemp*│ 1  0 0	│  , Tc = RcTemp*│-FV_CalibBoardY	│+TcTemp
    //				└ 0  0 1	┘				  └     0			┘
    Rc[0]= RcTemp[1], 	Rc[1]=-RcTemp[0], 	Rc[2]= RcTemp[2];
    Rc[3]= RcTemp[4], 	Rc[4]=-RcTemp[3], 	Rc[5]= RcTemp[5];
    Rc[6]= RcTemp[7], 	Rc[7]=-RcTemp[6], 	Rc[8]= RcTemp[8];
    double FV_CalibBoard[3] = {FV_CalibBoardX,-FV_CalibBoardY,0};
    MatrixMultiplyVector(RcTemp, FV_CalibBoard,Tc);
    Tc[0] += TcTemp[0];
    Tc[1] += TcTemp[1];
    Tc[2] += TcTemp[2];

    ParamFile>>kc[0]>>kc[1]>>kc[2]>>kc[3]>>kc[4];
    ParamFile.close();
    return true;
}
void C3DConstruct::PixelPos2CamNormPos(int PixelPosX, int PixelPosY, double CamNormPos[2])
{//认为罗技摄像头没有畸变
    CamNormPos[0]=KKinvers[0]*PixelPosX+KKinvers[1]*PixelPosY+KKinvers[2];
    CamNormPos[1]=KKinvers[3]*PixelPosX+KKinvers[4]*PixelPosY+KKinvers[5];
}
void C3DConstruct::CamNormPos2BodyPos(double CamNormPos[2], double BodyPos[2], int FV_BallRadius)
{
    //因为k*[CamNormPosX,CamNormPosY,1]=Rc*[BodyPosX,BodyPosY,BodyPosZ]+Tc
    //且已知BodyPosZ=FV_BallRadius
    //所以[BodyPosX,BodyPosY,k]可以求解
    double X[3];//即[BodyPosX,BodyPosY,k]
    double A[9] = {	-Rc[0],	-Rc[1],	CamNormPos[0],
                    -Rc[3],	-Rc[4],	CamNormPos[1],
                    -Rc[6],	-Rc[7],		1,		};
    double B[3] = {	Rc[2]*FV_BallRadius+Tc[0],
                    Rc[5]*FV_BallRadius+Tc[1],
                    Rc[8]*FV_BallRadius+Tc[2]};
    double Ainv[9];
    InversMatrix(A,Ainv);
    MatrixMultiplyVector(Ainv,B,X);
    BodyPos[0] = X[0];
    BodyPos[1] = X[1];
}

