#include "nubot/omni_vision/ballfinder.h"
#include "ros/ros.h"

using namespace nubot;

BallFinder::BallFinder(Transfer & _transfer)
{
  transfer_ = &_transfer;
  is_detected_ball_=false;
}

BallFinder::~BallFinder()
{

}

bool
BallFinder::Process(cv::Mat &_image,const DPoint & _location, const Angle & _angle )
{
  is_detected_ball_= false;
  if(_image.cols==0 || _image.rows==0)
     return false;
  if(_image.cols!=check_flag_.cols || _image.rows!=check_flag_.rows)
       check_flag_.create(_image.rows,_image.cols,CV_8UC1);

  std::vector<ImageArea> candidate_areas;
  is_detected_ball_=RegionSearch(_image, candidate_areas);

  bool ball_in_field=false;
  if(is_detected_ball_)
  {
      size_t numstrans = candidate_areas.size();
      for(size_t i = 0 ; i < numstrans ; i++)
      {
          ball_area_ = candidate_areas[i];
          DPoint2i ball_pos(ball_area_.area_center_.x, ball_area_.area_center_.y);
          transfer_->calculateRealCoordinates(ball_pos,ball_real_loc_,false);
          //the coordinate of the ball don't touch the filed,so we coorect the location;
          int ball_pix_height = -0.0348*ball_real_loc_.radius_+35.6921;//这是估计出的 足球像素高度与其距离的关系，其中距离为第一次观测距离（而不是实际距离）
          PPoint calib_pixels(ball_real_loc_.angle_.radian_,ball_pix_height/4);//假设只颜色分割出了足球的一半，则需要校正这些像素
          ball_pos = ball_pos - DPoint2i(calib_pixels);
          transfer_->calculateRealCoordinates(ball_pos,ball_real_loc_,false);
          ball_real_loc_.radius_ = ball_real_loc_.radius_*(78.0-11.0)/78.0;//由于识别的是足球的质心，所以要进行校正，得到足球着地点(其中78是镜面高度，11是足球半径)(之所以不在颜色识别阶段就直接校正到着地点，是因为，理论上是看不到着地点的)
          DPoint pt =DPoint(ball_real_loc_);
          transfer_->correct_offset(pt);
          ball_real_loc_ = PPoint(pt);
          transfer_->calculateWorldCoordinates(ball_real_loc_ , _location,_angle, ball_global_loc_);
          double ball2robot  = ball_pos.distance(transfer_->omni_img_->getBigROI().center_);
          bool is_ball=true;
          if(ball2robot<120 && ball_area_.area_size_<70)
                 is_ball=false;
          if(ball_area_.area_size_/double(ball_area_.area_rect_.height* ball_area_.area_rect_.width) <0.2)
                is_ball=false;
          double ratio = ball_area_.area_rect_.height/double(ball_area_.area_rect_.width);
          if(ratio<0.2 || ratio>5)
                is_ball=false;
          if(field_info_.isInInterRect(ball_global_loc_,100) && is_ball)
          {
                 ball_in_field=true;
                 break;
          }
      }
  }
  is_detected_ball_ = ball_in_field;
  return is_detected_ball_;
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
  std::vector<nubot::ImageArea> area_list;//once a candidate area is found, it will be pushed in this list

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
          current_area.area_center_ = cv::Point(x_sum/current_area.area_size_,y_sum/current_area.area_size_);
          current_area.area_rect_ = cv::Rect(area_rect_Xmin,area_rect_Ymin,area_rect_Xmax-area_rect_Xmin+1,area_rect_Ymax-area_rect_Ymin+1);
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

  std::sort(area_list.begin(),area_list.end(),
            [](const nubot::ImageArea & area1, const nubot::ImageArea & area2){return (area1.area_size_ > area2.area_size_);});
  _target_areas = area_list;
  return(true);
}

DPoint
BallFinder::get_ball_global_loc()  {return ball_global_loc_;}
PPoint
BallFinder::get_ball_real_loc()    {return ball_real_loc_;}

void
nubot::BallFinder::showBall(cv::Mat & _img)
{
    if(is_detected_ball_)
    {
        std::vector<cv::Point>  & edge_pts = ball_area_.edge_points_;
        size_t numstrans= edge_pts.size();
        for(size_t i=0 ;i<numstrans; i++)
            cv::circle(_img, edge_pts[i], 1, cv::Scalar(0,255,255),1,8,0);
        imshow("image_info",_img);
        cv::waitKey(5.0);
    }
}
void
nubot::BallFinder::showBall(cv::Mat & _img, DPoint _robot_loc, Angle _angle,int filed_length,int filed_width)
{
    if(is_detected_ball_)
    {
      float Xrate=(float)(_img.cols*1.0/filed_length);
      float Yrate=(float)(_img.rows*1.0/filed_width);
      cv::circle(_img,cv::Point(int((ball_global_loc_.x_ + filed_length/2.0)*Xrate),
                             int((filed_width -(ball_global_loc_.y_ +filed_width/2.0))*Yrate)),
                             3,cv::Scalar(0,255,255),5,8,0);
      imshow("real_info",_img);
      cv::waitKey(5.0);
    }
}


