#include "nubot/omni_vision/whites.h"

using namespace nubot;
Whites::Whites(ScanPoints & _scanpts,Transfer & _coor_transfer)
{
   scanpts_   = &_scanpts;
   transfer_ =&_coor_transfer;
  
   img_white_.reserve(1000);
   
   h_low_=45;
   h_high_=120;
   nums_pts_line_=5;
   filter_width_=2;          
   merge_wave_=3;

   int I_max=200;
   int I_min=20;
   int T_max=30;
   int T_min=10;
   memset(t_new,0,256*sizeof(float));
   for (int i=0;i<I_min;i++)
	   t_new[i]=float(T_min);
   for (int i=I_min;i<=I_max;i++)
	   t_new[i]=float((cos(float(i-I_min)*SINGLEPI_CONSTANT/float(I_max-I_min)+SINGLEPI_CONSTANT)+1)*(T_max-T_min)/2+T_min);
   for (int i=I_max+1;i<256;i++)
	   t_new[i]=float(T_max);
}
void
Whites::showWhitePoints(cv::Mat & _img)
{
	size_t numstrans=img_white_.size();
	for(size_t i=0 ;i<numstrans; i++)
        cv::circle(_img,cv::Point(img_white_[i].x_,img_white_[i].y_),1,cv::Scalar(255,0,0),2,8,0);
    nubot::Circle Big_ROI = scanpts_->omni_img_->getBigROI();
    nubot::Circle Small_ROI =scanpts_->omni_img_->getSmallROI();
    cv::circle(_img,cv::Point(Big_ROI.center_.x_,Big_ROI.center_.y_),Big_ROI.radius_,cv::Scalar(0,0,255),1,8,0);
    cv::circle(_img,cv::Point(Small_ROI.center_.x_,Small_ROI.center_.y_),Small_ROI.radius_,cv::Scalar(0,0,255),1,8,0);
    imshow("image_info",_img);
    cv::waitKey(5.0);
}
void
Whites::showWhitePoints(cv::Mat & _img, DPoint _robot_loc, Angle _angle,int filed_length,int filed_width)
{
    float Xrate=(float)(_img.cols*1.0/filed_length);
    float Yrate=(float)(_img.rows*1.0/filed_width);
    std::vector<DPoint> world_pts;
    transfer_->calculateWorldCoordinates(robot_white_,_robot_loc,_angle,world_pts);
    int numstrans = robot_white_.size();
    cv::circle(_img,cv::Point(int((_robot_loc.x_ + filed_length/2.0)*Xrate),
                             int((filed_width -(_robot_loc.y_ +filed_width/2.0))*Yrate)),
                             5,cv::Scalar(255,0,255),10,8,0);
    for(size_t i=0 ;i<numstrans; i++)
    {
        if(std::abs(world_pts[i].x_)< filed_length/2.0  && std::abs(world_pts[i].y_) < filed_width/2.0)
            cv::circle(_img,cv::Point(int((world_pts[i].x_ + filed_length/2.0)*Xrate),
                                     int((filed_width -(world_pts[i].y_ +filed_width/2.0))*Yrate)),
                                     1,cv::Scalar(255,0,0),2,8,0);
    }
    imshow("real_info",_img);
    cv::waitKey(5.0);
}

void
Whites::detectWave(std::vector<uchar> & colors,std::vector<bool> & wave_hollow,std::vector<bool> & wave_peak)
{
	int Line_Length=int(colors.size());
	for(int i=1;i<Line_Length-1;i++)     
	{
        if (colors[i]>colors[i+1] && colors[i]>=colors[i-1])
		{
			wave_peak[i]=true;                    
			for (int j=1;j<=merge_wave_;j++)      
			{
				if (i-j>=0)
				{
                    if (wave_peak[i-j]&&std::abs(double(colors[i-j])-double(colors[i]))<10)
					{
						if(colors[i]<colors[i-j])
							wave_peak[i]=false;
						else
							wave_peak[i-j]=false;
						for (int k=1; k<=j; k++)
						{
							wave_hollow[i-k]=false;
						}
					}
				}
			}
		}
        if (colors[i+1]>=colors[i]  && colors[i]< colors[i-1])
		{
			wave_hollow[i]=true;                                   
		}
	}
}

void 
Whites::findNearHollow(vector<uchar> & colors,vector<bool> & wave_peak,vector<bool> & wave_hollow,vector<peak> & peak_count)
{
	peak Peak;
	bool need_find_left=true;
	bool need_find_right=true;
	int Step;
	int Line_Length=int(wave_peak.size());
	for(int i=0;i<Line_Length;i++)    
	{
		if (wave_peak[i])
		{
			Peak.peak_index   = i;
			Peak.left_hollow  = MIN_NUMBRT_CONST;
			Peak.right_hollow = MAX_NUMBRT_CONST;
			need_find_left=need_find_right=true;
			Step=1;
			while (need_find_left || need_find_right)
			{
				if (i-Step>=0 && need_find_left)   
				{
					if (wave_hollow[i-Step])
					{
						Peak.left_hollow=i-Step;
						need_find_left=false;
					}
				}
				if (i-Step<0)
				{
					Peak.left_hollow=MIN_NUMBRT_CONST;
					need_find_left=false;
				}

				if (i+Step<Line_Length && need_find_right)
				{
					if (wave_hollow[i+Step])
					{
						Peak.right_hollow=i+Step;
						need_find_right=false;
					}
				}
				if (i+Step>=Line_Length)
				{
					need_find_right=false;
					Peak.right_hollow=MAX_NUMBRT_CONST;
				}
				Step++;
			}
			if(Peak.left_hollow!=MIN_NUMBRT_CONST||Peak.right_hollow!=MAX_NUMBRT_CONST)    
			{
				Peak.near_hollow=std::abs(Peak.left_hollow-i)<std::abs(Peak.right_hollow-i) ? Peak.left_hollow : Peak.right_hollow;
				if (std::abs(Peak.left_hollow-i)==std::abs(Peak.right_hollow-i))
				{
					if (colors[Peak.left_hollow]>=colors[Peak.right_hollow])
						Peak.near_hollow=Peak.left_hollow;
					else
						Peak.near_hollow=Peak.right_hollow;
				}
				Peak.width=std::abs(Peak.peak_index-Peak.near_hollow);
				if(Peak.peak_index+Peak.width<Line_Length && Peak.peak_index-Peak.width>0)
                    Peak.boundaries=true;
				else 
					Peak.boundaries=false;
				peak_count.push_back(Peak);
			}	
		}	
	}
}
bool
Whites::IsWhitePoint(std::vector<DPoint2i> & _pts,double _color_average,vector<uchar> & _colors,peak & _peaks)
{
	double H_left=0;
	double H_right=0;
	int nwhites(0);
    double aver_colors=_color_average;
    cv::Mat & brg_image= scanpts_->omni_img_->bgr_image_;

    if ( _peaks.boundaries && _colors[_peaks.peak_index]>=aver_colors) //_peaks.width>5 &&
	{   
        if (std::abs(double(_colors[_peaks.peak_index])-double(_colors[_peaks.peak_index+_peaks.width]))>=t_new[int(aver_colors)]
         && std::abs(double(_colors[_peaks.peak_index])-double(_colors[_peaks.peak_index-_peaks.width]))>=t_new[int(aver_colors)])
		{
			if (_peaks.left_hollow!=MIN_NUMBRT_CONST)
			{
                 cv::Vec3b bgr   =brg_image.at<cv::Vec3b>(cv::Point(_pts[_peaks.left_hollow].x_,_pts[_peaks.left_hollow].y_));
                 cv::Vec3b color_hsv=scanpts_->omni_img_->bgr2hsv(bgr);
                 H_left=color_hsv[0];
			}	
			if (_peaks.right_hollow!=MAX_NUMBRT_CONST)
			{
                cv::Vec3b bgr  = brg_image.at<cv::Vec3b>(cv::Point(_pts[_peaks.right_hollow].x_,_pts[_peaks.right_hollow].y_));
                cv::Vec3b color_hsv=scanpts_->omni_img_->bgr2hsv(bgr);
				H_right=color_hsv[0];
			}
            if (H_left>=h_low_ -20&& H_left<=h_high_ +40/*&& H_right>=h_low_&& H_right<=h_high_&&std::abs(H_left-H_right)<=80*/)
			{
                if ((_colors[_peaks.peak_index-_peaks.width]>=0.45*aver_colors &&
                     _colors[_peaks.peak_index+_peaks.width]>=0.45*aver_colors)   ||
                    (_colors[_peaks.peak_index-_peaks.width]>=0.45*100 &&
                     _colors[_peaks.peak_index+_peaks.width]>=0.45*100))
				{		
					if (nwhites<nums_pts_line_&& _colors[_peaks.peak_index]>150)
					{
						nwhites++;
						return true;
					}							
				}
			}

		}
	}
	return false;
}
void
Whites::detectWhitePts(std::vector<DPoint2i> & pts,std::vector<uchar> & _ColorY_Aver,std::vector<DPoint2i> & whites)
{
    whites.clear();
    whites.reserve(10);
    CV_Assert(pts.size()==_ColorY_Aver.size());
	int Line_Length=pts.size();
	double ColorY_Aver_Sum(0);  

	std::vector<bool> wave_peak(Line_Length,false);
	std::vector<bool> wave_hollow(Line_Length,false); 
	std::vector<peak> peak_count;
	peak_count.reserve(Line_Length);

	for(int i=0; i<Line_Length; i++)         
        ColorY_Aver_Sum=ColorY_Aver_Sum+_ColorY_Aver[i];
    ColorY_Aver_Sum=ColorY_Aver_Sum/Line_Length;
    detectWave(_ColorY_Aver,wave_hollow,wave_peak);

    findNearHollow(_ColorY_Aver,wave_peak,wave_hollow, peak_count);

	size_t numstrans=peak_count.size();
	for(size_t i=0;i<numstrans;i++)    
 	{		
        if(IsWhitePoint(pts,ColorY_Aver_Sum,_ColorY_Aver,peak_count[i]))
            whites.push_back(pts[peak_count[i].peak_index]);
 	}
}

void 
Whites::calculateWeights()
{
	weights_.clear();
	double ref2=150.0*150.0;
	double d2=250.0*250.0;
	double tmpweight(0);
    double distofeature(0);
	size_t numtrans=robot_white_.size();
    weights_.resize(numtrans);
#ifdef using_openmp
#pragma omp parallel for private(tmpweight,distofeature) shared(ref2,d2)
#endif
	for(size_t index=0; index < numtrans; index++)
	{
        distofeature=robot_white_[index].radius_*robot_white_[index].radius_;
		tmpweight=(ref2+d2)/(d2+distofeature);
        weights_[index]= tmpweight;
	}
}

void
Whites::process()
{
	weights_.clear();
	img_white_.clear();
	robot_white_.clear();
    std::vector< std::vector<DPoint2i> > whites;

	size_t nums_polars=scanpts_->polar_pts_.size();
    whites.resize(nums_polars);
#ifdef using_openmp
#pragma omp parallel for
#endif
	for(size_t i = 0; i < nums_polars; i++)
        detectWhitePts(scanpts_->polar_pts_[i],scanpts_->filter_polar_pts_y_[i],whites[i]);
    for(size_t i = 0; i < nums_polars; i++)
       for(size_t j =0 ; j <whites[i].size(); j++)
           img_white_.push_back(whites[i][j]);


    size_t nums_horizs=scanpts_->horiz_pts_.size();
    whites.clear();
    whites.resize(2*nums_horizs);
#ifdef using_openmp
#pragma omp parallel for
#endif
	for(size_t i = 0; i < nums_horizs ; i++)
	{
        detectWhitePts(scanpts_->horiz_pts_[i],scanpts_->filter_horiz_pts_y_[i],whites[i*2]);
        detectWhitePts(scanpts_->verti_pts_[i],scanpts_->filter_verti_pts_y_[i],whites[i*2+1]);
	}
    for(size_t i = 0; i < 2*nums_horizs; i++)
       for(size_t j =0 ; j <whites[i].size(); j++)
           img_white_.push_back(whites[i][j]);

    transfer_->calculateRealCoordinates(img_white_,robot_white_);
	calculateWeights();
}
