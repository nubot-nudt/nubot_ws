#include "nubot/omni_vision/glocalization.h"

using namespace nubot;

Globallocalization::Globallocalization(Optimise & _optimise)
{
	samples_.reserve(315);
	opti_=&_optimise;
}

void 
Globallocalization::rankErrorsforSamples(const std::vector<double> & _weights,const std::vector<DPoint> &  _white_pt,Angle & _angle,int _times)
{
	CV_Assert(_times<3&&_times>=0);
	samples_.clear();
	int startx = opti_->startx_+_times*35;  
	int starty = opti_->starty_+_times*35;
	int endx   = opti_->endx_;
	int endy   = opti_->endy_;
	location_err tmpLocErr;

	for (int n=starty;n<=endy;n=n+100)                
	{                      
		for(int m=startx;m<=endx;m=m+100)       
		{
			tmpLocErr.loction=DPoint(m,n);
			tmpLocErr.ang=_angle;
			tmpLocErr.error=opti_->caculateErrors(_weights,_white_pt,tmpLocErr.loction,_angle);
	        samples_.push_back(tmpLocErr);
		}
	}
	if(_times==1)
	{
		for (int n=starty;n<=endy;n=n+100)  
		{
			tmpLocErr.loction=DPoint(opti_->endx_,n);
			tmpLocErr.ang=_angle;
            tmpLocErr.error=opti_->caculateErrors(_weights,_white_pt,tmpLocErr.loction,_angle);
			samples_.push_back(tmpLocErr);
		}
		for (int m=startx;m<=endx;m=m+100)  
		{
			tmpLocErr.loction=DPoint(m,opti_->endy_);
			tmpLocErr.ang=_angle;
			tmpLocErr.error=opti_->caculateErrors(_weights,_white_pt,tmpLocErr.loction,_angle);
			samples_.push_back(tmpLocErr);

		}
		tmpLocErr.loction=DPoint(opti_->endx_,opti_->endy_);
		tmpLocErr.ang=_angle;
        tmpLocErr.error=opti_->caculateErrors(_weights,_white_pt,tmpLocErr.loction,_angle);
		samples_.push_back(tmpLocErr);
	}
	if(_times==2)
	{
		for (int n=starty;n<=endy;n=n+100)  
		{
			tmpLocErr.loction=DPoint(opti_->startx_,n);
			tmpLocErr.ang=_angle;
			tmpLocErr.error=opti_->caculateErrors(_weights,_white_pt,tmpLocErr.loction,_angle);
			samples_.push_back(tmpLocErr);
		}
		for (int m=startx;m<=endx;m=m+100)  
		{
			tmpLocErr.loction=DPoint(m,opti_->starty_);
			tmpLocErr.ang=_angle;
			tmpLocErr.error=opti_->caculateErrors(_weights,_white_pt,tmpLocErr.loction,_angle);
			samples_.push_back(tmpLocErr);
		}		
		tmpLocErr.loction=DPoint(opti_->startx_,opti_->starty_);
		tmpLocErr.ang=_angle;
		tmpLocErr.error=opti_->caculateErrors(_weights,_white_pt,tmpLocErr.loction,_angle);
		samples_.push_back(tmpLocErr);
	}	 
     std::partial_sort(samples_.begin(), samples_.begin()+5,samples_.end(),
                       [](location_err &loc1,location_err & loc2){ return (loc1.error<loc2.error);});
}

bool
Globallocalization::process(const std::vector<double> & _weights,const std::vector<PPoint> & _white_pt,DPoint & _robotloction, Angle & _angle)
{
	CV_Assert(_weights.size()==_white_pt.size());
    std::vector< DPoint > real_white;
    real_white.resize(_white_pt.size());
    for(int j = 0 ; j <_white_pt.size(); j++)
        real_white[j] = DPoint(_white_pt[j]);
    _angle = Angle(0);
	static int times(0);
	static std::vector<location_err> tmpRecord(3);

    rankErrorsforSamples(_weights,real_white,_angle,times);
	DPoint tempglobalpos(100000,100000),tempglobalposold(100000,100000);            
	Angle  tempglobalangle(0.0),tempglobalangleold(0);
	double referror(-1000),referrorold(10000);
	size_t numtrans=_weights.size(); 
	bool IsGlobalAgain(false);
	
	for(size_t j = 0; j < 5; j++)
	{
		tempglobalpos=samples_[j].loction;
		tempglobalangle=samples_[j].ang; 
        referror=opti_->process(_weights,real_white,tempglobalpos,tempglobalangle);
		if(referror<referrorold)
		{
			referrorold=referror;
            tempglobalposold=tempglobalpos;
			tempglobalangleold=tempglobalangle;
		}
	}
	referror=referrorold/numtrans;
	tempglobalpos=tempglobalposold;
	tempglobalangle=tempglobalangleold;

	if(numtrans<=15)  
	{
	   IsGlobalAgain=true;
	   times=0;
	}
	else
	{
		tmpRecord[times].loction=tempglobalpos;
		tmpRecord[times].ang=tempglobalangle;
		tmpRecord[times].error=referror;
		if(times<2)
			IsGlobalAgain=true;
		else
		{
			int tempindex=0;
			for(int i=1;i<3;i++)
			{
				if (tmpRecord[i].error<=tmpRecord[tempindex].error)
					tempindex=i;	   
			}    
			tempglobalpos   = tmpRecord[tempindex].loction;
			tempglobalangle = tmpRecord[tempindex].ang;
			IsGlobalAgain   = false;
		}	
		if(times==2)
		  times=0;
		else
		  times++;
	}
	_robotloction=tempglobalpos;
	_angle=tempglobalangle;
	return IsGlobalAgain;
}
