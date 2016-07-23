#include "nubot/omni_vision/obstacles.h"

using namespace nubot;

Obstacles::Obstacles(ScanPoints & _scanpts,Transfer & _trans)
{
    scanpts_=&_scanpts;
    transfer_=& _trans;
    size_t numtrans=scanpts_->polar_pts_.size();
    obsthres_=50;
    obstacle_length_thres_ = 0.3;
    obstacle_basic_thres_  = 0.05;
    interval_radian_=DOUBLEPI_CONSTANT/numtrans;
    weight_.resize(numtrans);
    distance_.resize(numtrans);
    black_pts_.resize(numtrans);
    obs_segments_.resize(numtrans);
    real_obstacles_.reserve(OBS_MAXDISTANCE_CONST);
    world_obstacles_.reserve(OBS_MAXDISTANCE_CONST);
    for(size_t i=0 ; i<numtrans; i++)
    {
        black_pts_[i].reserve(scanpts_->polar_pts_[i].size());
        obs_segments_[i].reserve(scanpts_->polar_pts_[i].size());
    }
}

/**  @brief   the main function of obstacles detection
 *   @return  the number of obstacles detected
 *   @author  Dan Xiong
 *   @see     detectblacks,getweights,getobstacles
 *   @date    2013.12.27*/
void
Obstacles::process(cv::Mat & _segment_result,DPoint & _robot_loc,Angle & _robot_head)
{
    real_obstacles_.clear();
    world_obstacles_.clear();
    size_t numtrans=scanpts_->polar_pts_.size();
    //uchar miniY=*min_element(_ColorY.begin(),_ColorY.end());
    //uchar miniY=*max_element(_ColorY.begin(),_ColorY.end());
    miniY_=255;maxY_=0;
    for(size_t i=0; i< numtrans ;i++)
    {
        obs_segments_[i].clear();
        std::vector<DPoint2i> & polar_pts= scanpts_->polar_pts_[i];
        std::vector<uchar>    & ColorY   = scanpts_->polar_pts_y_[i];
        size_t Line_Length=polar_pts.size();
        for(size_t j = 0 ; j < Line_Length ; j++)
        {
            if(ColorY[j]<miniY_ && ColorY[j] !=0)  miniY_ = ColorY[j];
            if(ColorY[j]>maxY_  && ColorY[j] !=0)  maxY_  = ColorY[j];
            if(_segment_result.at<unsigned char>(polar_pts[j].y_,polar_pts[j].x_)==VISION_COLORSEGMENT_BLACK)
                obs_segments_[i].push_back(true);
            else
                obs_segments_[i].push_back(false);
        }
    }
    if(miniY_>OBS_MINDARKNESS_CONST)
        return  ;
    for(size_t i=0;i<numtrans;i++)
        detectBlacks(scanpts_->polar_pts_[i],scanpts_->filter_polar_pts_y_[i],black_pts_[i],obs_segments_[i]);

    getWeights();
    getObstacles();
    transferCoordinate(_robot_loc,_robot_head);
}

void 
Obstacles::detectBlacks(std::vector<DPoint2i> & _pts,std::vector<uchar> & _ColorY,
                        std::vector<DPoint2i> & _black_pts,std::vector<bool> & obs_segment)
{
    _black_pts.clear();
    size_t Line_Length=_pts.size();
    double Yfirst = 0;
    int cnt = 0;
    for(size_t i=0; i < Line_Length; i++)//obsthres_)
    {
        if (_ColorY[i]!=0 && _ColorY[i]< miniY_+maxY_*(obstacle_basic_thres_+obstacle_length_thres_*double(i)/double(Line_Length)))
        {
            if (cnt==0)
            {
                Yfirst = i;
                _black_pts.clear();
            }
            _black_pts.push_back(_pts[i]);
            ++cnt;
            if (Yfirst!=0 && cnt>OBS_BLACKCOUNTFACTOR/Yfirst) /*found enough black points*/
            {
                if (i-Yfirst<OBS_BLACKSIZE) //_black_pts.push_back(_pts[Yfirst]);//obspoints[i] = Yfirst;/*enough black points close together*/
                break; /*goto the next ruler*/
            }
        }
        else if (cnt>0) cnt--; /*to be robust for small black spickles*/
        if (Yfirst!=0 && cnt>0 && i-Yfirst>OBS_BLACKCOUNTFACTOR/Yfirst) /*found enough black points*/
        {
            if (OBS_BLACKSIZE*(i-Yfirst)<cnt)// _black_pts.push_back(_pts[Yfirst]);//obspoints[i] = Yfirst;/*enough black points close together*/
            break; /*goto the next ruler*/
        }
    }
    if (cnt>0 && _black_pts.size()==0) /*set not finished object, probably far away*/
       _black_pts.push_back(_pts[Yfirst]);
    if(cnt<=0)
        _black_pts.clear();
     /*   if(_ColorY[i] !=0 && _ColorY[i]<obsthres_ )// && obs_segment[i])
            _black_pts.push_back(_pts[i]);*/
}
void 
Obstacles::getWeights()
{
   /* size_t numtrans=black_pts_.size();
    DPoint2d weight_center(0,0);
    Circle Image_ROI=scanpts_->omni_img_->getBigROI();
    for(size_t i=0;i<numtrans;i++)
    {
        weight_[i]   = DBL_MIN;
        distance_[i] = DBL_MAX;
        std::vector<DPoint2i> & pts=black_pts_[i];
        size_t nums_black=pts.size();
        if(nums_black>0)
        {
            weight_center=DPoint2d(0,0);
            for(size_t j=0 ;j<nums_black;j++)
                weight_center+=DPoint2d(pts[j]);
            weight_center=1.0/nums_black*weight_center;
            double center_dis=weight_center.distance(Image_ROI.center_);
            weight_[i]= double(nums_black)/scanpts_->polar_pts_[i].size() * std::exp(-center_dis/Image_ROI.radius_);
            if(weight_[i]>OBS_WEIGHT_THRES_CONST)
                transfer_->calculateRealDistance(pts[0],distance_[i]);
        }
    }*/
    size_t numtrans=black_pts_.size();
    Circle Image_ROI=scanpts_->omni_img_->getBigROI();
    for(size_t i=0;i<numtrans;i++)
    {
       distance_[i] = DBL_MAX;
       std::vector<DPoint2i> & pts=black_pts_[i];
       size_t nums_black=pts.size();
       if(nums_black>0)
            transfer_->calculateRealDistance(pts[0],distance_[i]);
    }
}

void
Obstacles::getObstacles()
{
    int numtrans=weight_.size();
    int rulerupper,rulerlower;
    int numobstacles(0);
    for (int j=0; j<numtrans; ++j)
    {
        int closestobs(0);
        for (int i=0; i<numtrans; ++i)
        {
            if(distance_[i]<distance_[closestobs])
                closestobs = i;
        }
        double min_distance = distance_[closestobs];
        if (min_distance>OBS_MAXDISTANCE_CONST) break;
        /*from TU/E code*/
        int flag(0);
        for (rulerupper=closestobs;rulerupper<closestobs+numtrans;++rulerupper)
        {
            if (std::abs(distance_[rulerupper%numtrans]-min_distance)>=OBS_PARTITION_CONST)
            {
                if (flag==2) break; else flag++;
            }
            else {flag=0;}
        }
        flag=0;
        for (rulerlower=closestobs;rulerlower>closestobs-numtrans;--rulerlower)
        {
            int ind = rulerlower%numtrans;
            if (ind<0) ind+=numtrans;
            if (std::abs(distance_[ind%numtrans]-min_distance)>=OBS_PARTITION_CONST)
            {
                if (flag==2) break; else flag++;
            }
            else {flag=0;}
        }

        /*compute obstacle parameters*/
        double obsphi   = (rulerupper+rulerlower)/2*interval_radian_;
        double obsdist  = min_distance;
        double obswidth = rulerupper-rulerlower-2;

        /*check size of obstacle(s)*/
        double width = obswidth*interval_radian_*obsdist;
        //determine amount of obstacles to split obstacle blob
        int n = int(0.5+width/2.0/OBS_RADIUS_CONST*1.1);
        //clip amount of subdivisions
        if (n>4) n=4;
        if (n<1) n=1;

        //determine new upper and lower limit (used for n>1)
        double philower = rulerlower*interval_radian_+1.*OBS_RADIUS_CONST/obsdist;
        double phiupper = rulerupper*interval_radian_-1.*OBS_RADIUS_CONST/obsdist;

        for (int i=0; i<n; ++i)
        {
            if (numobstacles<OBS_NUMBER_CONST)
            {
                //determine center of obstacle in case of n>1
                //! the angle has a bias
                if (n>1) obsphi = philower+(phiupper-philower)*i/(n-1.0);
                real_obstacles_.push_back(PPoint(Angle(obsphi),obsdist+OBS_RADIUS_CONST));
                ++numobstacles;
            }
        }

        for (int k=rulerlower;k<rulerupper;++k)
        {
            int ind = k%numtrans;
            if (ind<0) ind+=numtrans;
            distance_[ind] = DBL_MAX;
        }
    }
}

void
Obstacles::transferCoordinate(DPoint & _robot_loc, Angle & _robot_head)
{
    if(real_obstacles_.empty())
        return ;
    for(int i = 0 ;i < real_obstacles_.size();i++ )
    {
        DPoint pt =DPoint(real_obstacles_[i]);
        transfer_->correct_offset(pt);
        real_obstacles_[i] = PPoint(pt);
    }
    transfer_->calculateWorldCoordinates(real_obstacles_,_robot_loc,_robot_head, world_obstacles_);
    for(int i = 0 ; i < world_obstacles_.size() ; i++)
    {
        if(field_info_.isInOuterRect(world_obstacles_[i],0) || real_obstacles_[i].radius_ > 600)
        {
            world_obstacles_.erase(world_obstacles_.begin()+i);
            real_obstacles_.erase(real_obstacles_.begin()+i);
            i--;
        }
    }
}

void
Obstacles::setObsThres(int _obs_thres,double _obstacle_length_thres,double _obstacle_basic_thres){
    obsthres_ =  _obs_thres;
    obstacle_length_thres_= _obstacle_length_thres;
    obstacle_basic_thres_ = _obstacle_basic_thres;
}

void Obstacles::showObstacles(cv::Mat & _img)
{
    size_t numstrans=black_pts_.size();
    for(size_t i=0 ;i<numstrans; i++)
    {
      //  if(weight_[i]>OBS_WEIGHT_THRES_CONST)
        {
            std::vector<DPoint2i> & pts=black_pts_[i];
            size_t nums_black=pts.size();
            for(size_t j=0 ;j<nums_black;j++)
                _img.at<cv::Vec3b>(cv::Point(pts[j].x_,pts[j].y_))=cv::Vec3b(0,0,255);
        }
    }

    vector<DPoint2i> img_pts;
    transfer_->calculateImageCoordinates(real_obstacles_,img_pts);
    numstrans=img_pts.size();
    for(size_t i=0 ;i<numstrans; i++)
        cv::circle(_img,cv::Point(img_pts[i].x_,img_pts[i].y_),2,Scalar(255,255,0),4,8,0);
    nubot::Circle Big_ROI = scanpts_->omni_img_->getBigROI();
    nubot::Circle Small_ROI =scanpts_->omni_img_->getSmallROI();
    cv::circle(_img,cv::Point(Big_ROI.center_.x_,Big_ROI.center_.y_),Big_ROI.radius_,cv::Scalar(0,0,255),1,8,0);
    cv::circle(_img,cv::Point(Small_ROI.center_.x_,Small_ROI.center_.y_),Small_ROI.radius_,cv::Scalar(0,0,255),1,8,0);
    imshow("image_info",_img);
    cv::waitKey(5.0);
}
void Obstacles::showObstacles(cv::Mat & _img,DPoint _robot_loc, Angle _angle, int filed_length,int filed_width)
{
    float Xrate=(float)(_img.cols*1.0/filed_length);
    float Yrate=(float)(_img.rows*1.0/filed_width);
    cv::circle(_img,cv::Point(int((_robot_loc.x_ + filed_length/2.0)*Xrate),
                              int((filed_width -(_robot_loc.y_ +filed_width/2.0))*Yrate)),
               5,cv::Scalar(255,0,255),10,8,0);
    size_t numstrans = world_obstacles_.size();
    for(size_t i=0 ;i<numstrans; i++)
    {
        if(std::abs(world_obstacles_[i].x_)< filed_length/2.0  && std::abs(world_obstacles_[i].y_) < filed_width/2.0)
            cv::circle(_img,cv::Point(int((world_obstacles_[i].x_ + filed_length/2.0)*Xrate),
                                      int((filed_width -(world_obstacles_[i].y_ +filed_width/2.0))*Yrate)),
                       3,cv::Scalar(0,0,255),5,8,0);
    }
    imshow("real_info",_img);
    cv::waitKey(5.0);
}
