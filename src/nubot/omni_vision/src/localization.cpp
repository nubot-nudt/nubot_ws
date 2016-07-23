#include "nubot/omni_vision/localization.h"
#include "ros/ros.h"
using namespace nubot;

Localization::Localization(Optimise & _optimise)
{
	opti_     = &_optimise;
	variant_ang_=0.05;
	variant_pos_=DPoint2d(0.0,0.0);
}

void
Localization::kalmanfilter(DPoint _delta_pos,Angle _delta_ang,DPoint _pos_visual,Angle _ang_visual,DPoint2d _var_xy,double _var_phi,DPoint & _pos_odo,Angle & _facing_odo)
{
	kalmanfilter(_delta_pos,_delta_ang, _pos_odo,_facing_odo);
	DPoint robotlocation;
	Angle  robotangle;
	double v1, v2;

	v1=variant_pos_.x_;
	v2=_var_xy.x_;
	robotlocation.x_ = (v2*_pos_odo.x_+v1*_pos_visual.x_)/(v1+v2);
	variant_pos_.x_ = (v1*v2)/(v1+v2);
	v1=variant_pos_.y_;
	v2=_var_xy.y_;
	robotlocation.y_ = (v2*_pos_odo.y_+v1*_pos_visual.y_)/(v1+v2);
	variant_pos_.y_ = (v1*v2)/(v1+v2);

    v1=variant_ang_;
	v2=_var_phi;
	double radian_tmp=(v2*_facing_odo.radian_+v1*_ang_visual.radian_)/(v1+v2);
	if(std::abs(_facing_odo.radian_-_ang_visual.radian_)>SINGLEPI_CONSTANT)
	{
		if(_facing_odo.radian_>_ang_visual.radian_)
			radian_tmp+=v1*DOUBLEPI_CONSTANT/(v1+v2);
		else if(_ang_visual.radian_>_facing_odo.radian_)
		    radian_tmp+=v2*DOUBLEPI_CONSTANT/(v1+v2);
	}
    robotangle =Angle(radian_tmp);
    variant_ang_ = (v1*v2)/(v1+v2);
	_pos_odo=robotlocation;
    _facing_odo=robotangle;
}

void
Localization::kalmanfilter(DPoint _delta_pos,Angle  _delta_ang,DPoint & _pos_odo,Angle & _facing_odo )
{
	
	DPoint2d std_delta_pos=DPoint2d(std::max(double(std::abs(_delta_pos.x_)),5.0), std::max(double(std::abs(_delta_pos.y_)),5.0));
	double std_delta_head = std::max(double(std::abs(_delta_ang.radian_)),0.04);
	variant_pos_.x_+=0.375*std_delta_pos.x_*std_delta_pos.x_;
	_pos_odo.x_+=_delta_pos.x_;
	variant_pos_.y_+=0.375*std_delta_pos.y_*std_delta_pos.y_;
	_pos_odo.y_+=_delta_pos.y_;
	variant_ang_+=0.375*std_delta_head*std_delta_head;
	_facing_odo=_facing_odo+ _delta_ang;
 
}

void
Localization::process(const std::vector<double> & _weights,const std::vector<PPoint> & _white_pt,
                      DPoint & _robotloction, Angle & _angle,DPoint & _delta_loc,Angle & _delta_ang)
{

    CV_Assert(_weights.size()==_white_pt.size());
    int numtrans=_weights.size();
    std::vector< DPoint > real_white;
    real_white.resize(_white_pt.size());
    for(int j = 0 ; j <_white_pt.size(); j++)
        real_white[j] = DPoint(_white_pt[j]);
    DPoint pos_odo    =  _robotloction;
    Angle last_ang = _angle;
    Angle  facing_odo =  _angle;
    DPoint2d delta_pos = _delta_loc;
    Angle    delta_ang = _delta_ang;

    _robotloction = _robotloction+_delta_loc;
    _angle = _angle+_delta_ang;
	double referror=0.0;
	static bool first_optimise=false;
	static int cycles_since_reset=0;
	if (numtrans>15)
        referror=opti_->process(_weights,real_white,_robotloction,_angle);
    if(numtrans>20)     //  if (internal_alternatives && lines.objectlist.size()>=10)
	{
		DPoint offsets [] = { DPoint(60.0,0.0), DPoint(0.0,60.0), DPoint(-60.0,0.0), DPoint(0.0,-60.0) };
		if(!first_optimise)
		{
			first_optimise=true;
			offsets[0]=DPoint(120.0,0.0);
			offsets[1]=DPoint(0.0,120.0);
			offsets[2]=DPoint(-120.0,0.0);
			offsets[3]=DPoint(0.0,-120.0);
		}
		double min_error = referror;
		int min_index=-1;
		double stdhead = sqrt(variant_ang_);
		double sfac = 1.7-0.7/(cycles_since_reset+1.0);  
		for (size_t i=0; i<4; i++) 
		{
            srand(cv::getTickCount());
			DPoint trial_pos = pos_odo+offsets[i];
            Angle trial_heading = facing_odo+(stdhead*(rand()/32767.0-0.5)*4.0);
            double err = opti_->process(_weights,real_white,trial_pos,trial_heading);
			if (err<min_error && sfac*err<referror)
			{
				min_index=i;
				min_error=err;
                _robotloction=trial_pos;
                _angle=trial_heading;
				cycles_since_reset=0;
			}
		}
		if(min_index==-1)
			cycles_since_reset++;
	}

    double ddphi(0.0);
	DPoint2d ddxy(0.0,0.0);
    if (numtrans>15)
        opti_->analyse(_weights,real_white,ddxy, ddphi, _robotloction, _angle);
	double fnum = 16.0/(numtrans+4.0)+0.7; 
	DPoint2d trans_welt=delta_pos;
	double tempddxyx=std::abs(log(std::abs(ddxy.x_)+1e-6)+7);
	double tempddxyy=std::abs(log(std::abs(ddxy.y_)+1e-6)+7);
	DPoint2d var_xy=DPoint2d(trans_welt.y_*trans_welt.y_+fnum*225*(tempddxyx), 
		trans_welt.x_*trans_welt.x_+fnum*225*(tempddxyy)); 
    double var_phi = fnum*3.0461741978670859865/(ddphi*ddphi+1e-6);

    if (numtrans>15)
        kalmanfilter(delta_pos,delta_ang,_robotloction,_angle,var_xy,var_phi,pos_odo,facing_odo);
    else
        kalmanfilter(delta_pos,delta_ang,pos_odo,facing_odo);
    Angle delta_ang_tmp = last_ang -facing_odo;
    static Angle max_delta_ang(30,false);
    if(std::abs(delta_ang_tmp.degree()) > 30)
        facing_odo += delta_ang_tmp.radian_/std::abs(delta_ang_tmp.radian_)*max_delta_ang.radian_;
    tempglobalpos_   = _robotloction;
    tempglobalangle_ = _angle;
    _robotloction=pos_odo;
    _angle=facing_odo;
}
