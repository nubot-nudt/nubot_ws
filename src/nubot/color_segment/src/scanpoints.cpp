#include "scanpoints.h"

using namespace nubot;

ScanPoints::ScanPoints(OmniImage & _omni_img)
{
	omni_img_=&_omni_img;
    Circle  ImgROI    = omni_img_->getBigROI();
    int height  = omni_img_->getHeight();
    int width   = omni_img_->getWidth();

    filter_width_=2;

    polar_pts_.resize(360);
    polar_pts_y_.resize(360);
    filter_polar_pts_y_.resize(360);
	for(size_t i=0 ; i<360; i++)
	{
        polar_pts_[i]=scanPolarLine(Angle(double(i),false));
        polar_pts_y_[i].resize(polar_pts_[i].size());
        filter_polar_pts_y_[i].resize(polar_pts_[i].size());
	}

    interval_=1;
	ratio_=1;

    /*! @brief  guarantee the scanpoints in the image */
    DPoint2i relative_pt= DPoint2i(ratio_*ImgROI.radius_,ratio_*ImgROI.radius_);
    DPoint2i startpt    = DPoint2i(DPoint2i(ImgROI.center_)-relative_pt);
    int count= 0;
    while(startpt.x_ <= 0 || startpt.y_ <= 0)
    {
        count++;
        relative_pt = DPoint2i(ratio_*ImgROI.radius_-count,ratio_*ImgROI.radius_-count);
        startpt     = DPoint2i(DPoint2i(ImgROI.center_)-relative_pt);
    }

    /*! @brief  guarantee the scanpoints in the image */
    relative_pt     = DPoint2i(ratio_*ImgROI.radius_,ratio_*ImgROI.radius_);
    DPoint2i endpt  = DPoint2i(DPoint2i(ImgROI.center_)+relative_pt);
    count= 0;
    while(endpt.x_ >= width || endpt.y_ >= height)
    {
        count++;
        relative_pt = DPoint2i(ratio_*ImgROI.radius_-count,ratio_*ImgROI.radius_-count);
        endpt       = DPoint2i(DPoint2i(ImgROI.center_)+relative_pt);
    }


    int Point_Num_HV    = int(std::abs(endpt.x_-startpt.x_)/interval_);
    horiz_pts_.resize(Point_Num_HV);
    verti_pts_.resize(Point_Num_HV);
    horiz_pts_y_.resize(Point_Num_HV);
    verti_pts_y_.resize(Point_Num_HV);
    filter_horiz_pts_y_.resize(Point_Num_HV);
    filter_verti_pts_y_.resize(Point_Num_HV);

	for(int i=0;i<Point_Num_HV;i++)
	{
		DPoint2i tmpstart = DPoint2i(startpt.x_,startpt.y_+i*interval_);
		DPoint2i tmpend   = DPoint2i(endpt.x_,startpt.y_+i*interval_);
		while(tmpstart.distance(ImgROI.center_)>ImgROI.radius_)
			tmpstart.x_=tmpstart.x_+1;    
        while(tmpend.distance(ImgROI.center_)>ImgROI.radius_)
			tmpend.x_=tmpend.x_-1;   

        horiz_pts_[i] = scanHorizLine(tmpstart,tmpend);
        horiz_pts_y_[i].resize(horiz_pts_[i].size());
        filter_horiz_pts_y_[i].resize(horiz_pts_[i].size());

        tmpstart = DPoint2i(startpt.x_+i*interval_,startpt.y_);
        tmpend   = DPoint2i(startpt.x_+i*interval_,endpt.y_);
		while(tmpstart.distance(ImgROI.center_)>ImgROI.radius_)
		   tmpstart.y_=tmpstart.y_+1;    
		while(tmpend.distance(ImgROI.center_)>ImgROI.radius_)
		   tmpend.y_=tmpend.y_-1; 
        verti_pts_[i] = scanVertiLine(tmpstart,tmpend);
        verti_pts_y_[i].resize(verti_pts_[i].size());
        filter_verti_pts_y_[i].resize(verti_pts_[i].size());
	}

}


//! start point is image center and 
vector<DPoint2i>  
ScanPoints::scanPolarLine(const Angle & ang)
{

    Circle  ImgROI    = omni_img_->getBigROI();
    Circle  Small_ROI = omni_img_->getSmallROI();
    int height  = omni_img_->getHeight();
    int width   = omni_img_->getWidth();

    DPoint2i scanStartPt(PPoint(ang,Small_ROI.radius_));
    DPoint2i startPt = scanStartPt+DPoint2i(ImgROI.center_);

    /*! @brief  guarantee the scanpoints in the image */
    DPoint2i scanEndPt(PPoint(ang,ImgROI.radius_));
    DPoint2i endPt   = scanEndPt+DPoint2i(ImgROI.center_);
    int count= 0;
    while(endPt.x_ >= width || endPt.y_ >=height || endPt.x_< 0 || endPt.y_ < 0)
    {
        count++;
        scanEndPt = PPoint(ang,ImgROI.radius_-count);
        endPt   = scanEndPt+DPoint2i(ImgROI.center_);
    }

	return scanLine(startPt,endPt);
}
vector<DPoint2i>  
ScanPoints::scanVertiLine(const DPoint2i & startPt,const DPoint2i & endPt)
{
	return scanLine(startPt,endPt);
}
vector<DPoint2i>  
ScanPoints::scanHorizLine(const DPoint2i & startPt,const DPoint2i & endPt)
{
	return scanLine(startPt,endPt);
}

vector<DPoint2i>
ScanPoints::scanLine(const DPoint2i & startPt,const DPoint2i & endPt)
{ 

    vector<DPoint2i> polar_pt;
	polar_pt.reserve(int(endPt.distance(startPt)+20));
	Line_ scanLine(startPt,endPt);

	DPoint2i tmpScanpt=startPt;
	polar_pt.push_back(startPt);
	DPoint2i direction(endPt.x_>startPt.x_ ? 1:-1 , endPt.y_>startPt.y_ ? 1:-1);

	if (endPt.x_!=startPt.x_ && endPt.y_!=startPt.y_) 
	{	
		while(endPt.distance(tmpScanpt)>4)//! y increase faster than x
		{
            if(std::abs(scanLine.k_)>1)
			{
				tmpScanpt.y_=tmpScanpt.y_+direction.y_;
				double increase_x=(tmpScanpt.y_-scanLine.b_)/scanLine.k_;
				tmpScanpt.x_=int(increase_x+direction.x_*0.5); 		
			}
			else
			{
				tmpScanpt.x_=tmpScanpt.x_+direction.x_;
				double increase_y=scanLine.k_*tmpScanpt.x_+scanLine.b_;
				tmpScanpt.y_=int(increase_y+direction.y_*0.5); 
			}
			polar_pt.push_back(tmpScanpt);
		}
	}
	else if(startPt.x_!=endPt.x_ && startPt.y_==endPt.y_)
    {
        while (std::abs(endPt.x_-tmpScanpt.x_)>2)
		{
		    tmpScanpt.x_= tmpScanpt.x_+direction.x_;
			polar_pt.push_back(tmpScanpt);
		}
	}
	else if (startPt.x_==endPt.x_ && startPt.y_!=endPt.y_)
	{
        while (std::abs(endPt.y_-tmpScanpt.y_)>2)
	 	{			
			tmpScanpt.y_= tmpScanpt.y_+direction.y_;
			polar_pt.push_back(tmpScanpt);
		}
	}
	return polar_pt;
}

void
ScanPoints::process()
{
	size_t nums_polars=polar_pts_.size();
	for(size_t i = 0; i < nums_polars; i++)
	{
        vector<uchar> & ColorY=polar_pts_y_[i];
        omni_img_->getColorValues(polar_pts_[i],ColorY,0,SELECTION_YUV);	
	}
	size_t nums_horizs=horiz_pts_.size();
	for(size_t i = 0; i < nums_horizs ; i++)
	{
        std::vector<uchar> & horiz=horiz_pts_y_[i];
        omni_img_->getColorValues(horiz_pts_[i],horiz,0,SELECTION_YUV);
        std::vector<uchar> & verti=verti_pts_y_[i];
        omni_img_->getColorValues(verti_pts_[i],verti,0,SELECTION_YUV);
	}
    filter();
}

void
ScanPoints::filter()
{
    size_t nums_polars=polar_pts_.size();
    for(size_t i = 0; i < nums_polars; i++)
    {
        std::vector<uchar> & Polar_ColorY=polar_pts_y_[i];
        std::vector<uchar> & Aver_Polar_ColorY = filter_polar_pts_y_[i];
        int Line_Length=Polar_ColorY.size();
        Aver_Polar_ColorY.resize(Line_Length);
        for(int j=0; j< Line_Length; j++)
        {
            double Temp_sum_Y(0);
            int Temp_count(0);
            for (int k=-filter_width_;k<=filter_width_;k++)
            {
                if (k+j>=0 && k+j<Line_Length)
                {
                    Temp_count++;
                    Temp_sum_Y=Temp_sum_Y+(double)Polar_ColorY[k+j];
                }
            }
            Aver_Polar_ColorY[j]=uchar(Temp_sum_Y/double(Temp_count));
        }
    }
    size_t nums_horizs=horiz_pts_.size();
    for(size_t i = 0; i < nums_horizs ; i++)
    {
        std::vector<uchar> & horiz_ColorY=horiz_pts_y_[i];
        std::vector<uchar> & Aver_horiz_ColorY = filter_horiz_pts_y_[i];
        int Line_Length=horiz_ColorY.size();
        Aver_horiz_ColorY.resize(Line_Length);
        for(int j=0; j< Line_Length; j++)
        {
            double Temp_sum_Y(0);
            int Temp_count(0);
            for (int k=-filter_width_;k<=filter_width_;k++)
            {
                if (k+j>=0 && k+j<Line_Length)
                {
                    Temp_count++;
                    Temp_sum_Y=Temp_sum_Y+(double)horiz_ColorY[k+j];
                }
            }
            Aver_horiz_ColorY[j]=uchar(Temp_sum_Y/double(Temp_count));
        }

        std::vector<uchar> & verti_ColorY=verti_pts_y_[i];
        std::vector<uchar> & Aver_verti_ColorY = filter_verti_pts_y_[i];
        Line_Length=verti_ColorY.size();
        Aver_verti_ColorY.resize(Line_Length);
        for(int j=0; j< Line_Length; j++)
        {
            double Temp_sum_Y(0);
            int Temp_count(0);
            for (int k=-filter_width_;k<=filter_width_;k++)
            {
                if (k+j>=0 && k+j<Line_Length)
                {
                    Temp_count++;
                    Temp_sum_Y=Temp_sum_Y+(double)verti_ColorY[k+j];
                }
            }
            Aver_verti_ColorY[j]=uchar(Temp_sum_Y/double(Temp_count));
        }
    }
}

void
ScanPoints::showScanPoints()
{
  Circle  Small_ROI = omni_img_->getSmallROI();
  cv::Mat img=omni_img_->getBGRImage().clone();
  cv::circle(img,cv::Point(int(Small_ROI.center_.x_),int(Small_ROI.center_.y_)),1,Scalar(0,0,255),3,8,0);
  for(size_t i=0 ;i<polar_pts_.size(); i=i+5)
  {
	  std::vector<DPoint2i> & pts=polar_pts_[i];
      for(size_t j = 0; j < pts.size(); j=j++)
          img.at<cv::Vec3b>(cv::Point(pts[j].x_,pts[j].y_))=cv::Vec3b(255,255,0);
	  cv::putText(img,cv::format("%d", i),cv::Point(int(pts[pts.size()-1].x_),int(pts[pts.size()-1].y_)),
		           cv::FONT_HERSHEY_PLAIN, 0.8,Scalar(0,0,255), 1);
  }
  for(size_t i=0 ;i<horiz_pts_.size(); i=i+5)
  {
	  std::vector<DPoint2i> & pts=horiz_pts_[i];
      for(size_t j = 0; j < pts.size(); j=j++)
            img.at<cv::Vec3b>(cv::Point(pts[j].x_,pts[j].y_))=cv::Vec3b(255,0,255);
  }
  for(size_t i=0 ;i<verti_pts_.size(); i=i+5)
  {
	  std::vector<DPoint2i> & pts=verti_pts_[i];
      for(size_t j = 0; j < pts.size(); j=j++)
            img.at<cv::Vec3b>(cv::Point(pts[j].x_,pts[j].y_))=cv::Vec3b(0,255,255);
  }
  imshow("ScanPoints",img);
  cv::waitKey(5.0);
}

