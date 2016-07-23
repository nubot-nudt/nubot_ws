#include "whites.h"

using namespace nubot;
Whites::Whites(ScanPoints & _scanpts)
{
   scanpts_   = &_scanpts;

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
   t_new.assign(256,0);
   for (int i=0;i<I_min;i++)
	   t_new[i]=float(T_min);
   for (int i=I_min;i<=I_max;i++)
	   t_new[i]=float((cos(float(i-I_min)*SINGLEPI_CONSTANT/float(I_max-I_min)+SINGLEPI_CONSTANT)+1)*(T_max-T_min)/2+T_min);
   for (int i=I_max+1;i<256;i++)
	   t_new[i]=float(T_max);
   factor_ =1.7;
}


void
Whites::on_mouse(int event, int x, int y, int flags, void* tmpThis)
{
    Whites* PThis = (Whites*)tmpThis;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);
    if( event == CV_EVENT_LBUTTONDOWN )
    {
        cv::Mat img;
        cv::resize(PThis->image_show_,img,cv::Size(0,0),PThis->factor_,PThis->factor_,CV_INTER_NN);
        DPoint2i pt(x,y);
        pt = 1.0/PThis->factor_ * pt;
        size_t numstrans=PThis->image_whites_.size();
        double min_dis=1000000;
        int label=0;
        for(std::size_t i =0 ; i < numstrans ;i++)
        {
            double tmp_dis=PThis->image_whites_[i].distance(pt);
            if(min_dis > tmp_dis)
            {
                min_dis=tmp_dis;
                label=i;
            }
        }
        PThis->image_whites_.erase(PThis->image_whites_.begin()+label);
        PThis->img_white_=PThis->image_whites_;
        numstrans=PThis->image_whites_.size();
        for(size_t i=0 ;i<numstrans; i++)
            cv::circle(img,cv::Point(PThis->image_whites_[i].x_ *PThis->factor_ ,
                                     PThis->image_whites_[i].y_ *PThis->factor_),1,Scalar(0,0,255),2,8,0);
        imshow("WhitePoints",img);

    }
    if( event == CV_EVENT_RBUTTONDOWN )
    {
        cv::Mat img;
        cv::resize(PThis->image_show_,img,cv::Size(0,0),PThis->factor_,PThis->factor_,CV_INTER_NN);
        DPoint2i pt(x,y);
        pt = 1.0/PThis->factor_ * pt;
        PThis->image_whites_.push_back(pt);
        PThis->img_white_=PThis->image_whites_;
        size_t numstrans=PThis->image_whites_.size();
        for(size_t i=0 ;i<numstrans; i++)
            cv::circle(img,cv::Point(PThis->image_whites_[i].x_ *PThis->factor_,
                                     PThis->image_whites_[i].y_ *PThis->factor_),1,Scalar(0,0,255),1,8,0);
        imshow("WhitePoints",img);
    }
}

void
Whites::showWhitePoints()
{
    image_show_=scanpts_->omni_img_->getBGRImage().clone();
    cv::Mat img ;
    cv::resize(image_show_,img,cv::Size(0,0),factor_,factor_,CV_INTER_NN);
    cv::namedWindow("WhitePoints",1);
    cv::setMouseCallback( "WhitePoints", on_mouse, this);
    image_whites_=img_white_;
	size_t numstrans=img_white_.size();
	for(size_t i=0 ;i<numstrans; i++)
       cv::circle(img,cv::Point(img_white_[i].x_ * factor_ ,img_white_[i].y_ * factor_),1,Scalar(0,0,255),2,8,0);
    imshow("WhitePoints",img);
    cv::waitKey(0);
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

    if ( _peaks.boundaries && _colors[_peaks.peak_index]>=aver_colors) //_peaks.width>5
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
          //  if (H_left>=h_low_ -45 && H_left<=h_high_ +40 && H_right>=h_low_ -40 && H_right<=h_high_ + 120  && std::abs(H_left-H_right)<=80)
            if (H_left>=h_low_ && H_left<=h_high_)
            {
                if ((_colors[_peaks.peak_index-_peaks.width]>=0.45*aver_colors &&
                     _colors[_peaks.peak_index+_peaks.width]>=0.45*aver_colors)   ||
                    (_colors[_peaks.peak_index-_peaks.width]>=0.45*100 &&
                     _colors[_peaks.peak_index+_peaks.width]>=0.45*100))
				{		
                    if (nwhites < nums_pts_line_ && _colors[_peaks.peak_index]>150)
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
Whites::detectWhitePts(std::vector<DPoint2i> & pts,std::vector<uchar> & _ColorY_Aver)
{
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
			img_white_.push_back(pts[peak_count[i].peak_index]);
 	}
}


void
Whites::process()
{
	weights_.clear();
	img_white_.clear();
	robot_white_.clear();
	size_t nums_polars=scanpts_->polar_pts_.size();
	for(size_t i = 0; i < nums_polars; i++)
      detectWhitePts(scanpts_->polar_pts_[i],scanpts_->filter_polar_pts_y_[i]);
	size_t nums_horizs=scanpts_->horiz_pts_.size();
	for(size_t i = 0; i < nums_horizs ; i++)
	{
       detectWhitePts(scanpts_->horiz_pts_[i],scanpts_->filter_horiz_pts_y_[i]);
       detectWhitePts(scanpts_->verti_pts_[i],scanpts_->filter_verti_pts_y_[i]);
	}
}

