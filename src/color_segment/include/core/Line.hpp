#ifndef __NUBOT_CORE_LINE_HPP__
#define __NUBOT_CORE_LINE_HPP__

#include "PPoint.hpp"
#include <cmath>

namespace nubot
{

class Line_
{

public:
	// various constructors
	Line_();
	Line_(const Line_ & line);
	//!  general form of line
	Line_(double a,double b,double c);
	//!  line from slope k and intercept n
	Line_(double k,double b);
	//!  line from two points
	template<typename _Tp>  Line_(const DPoint_<_Tp> & pt1,const DPoint_<_Tp> & pt2);
	//!  line from slope k and a point
	template<typename _Tp>  Line_(double k,const DPoint_<_Tp> & pt);


	//!  whether the line is parallel
	bool isParallel(const Line_ & line) const;
	//!  whether the line is orthogonal 
	bool isOrthogonal (const Line_ & line) const;
	//!  the distance between a point and a line;
	template<typename _Tp> double distance(const DPoint_<_Tp> & pt) const ;
	//!  the distance between two parallel lines
	double distance(const Line_ & line) const ;
	//!  the cross point of two lines;
	DPoint_<double> crosspoint(const Line_ & line) const;

	double A_,B_,C_;  //< general form: Ax+By+C=0;
	double k_,b_;     //< slope and intercept form:Ax+By+C=0	    
	bool   isSlope_;  //< whether the slope is exist;
	bool   isLine_;   //< whether it is a line or not;
};


//////////////////////////////// Line ////////////////////////////////
inline Line_::Line_() : A_(0.),B_(0.),C_(0.),k_(0.),b_(0.),isSlope_(false),isLine_(false) {}
inline Line_::Line_(const Line_ & line) : A_(line.A_),B_(line.B_),C_(line.C_),
	k_(line.k_),b_(line.b_),isSlope_(line.isSlope_),isLine_(line.isLine_) {}
inline Line_::Line_(double a,double b,double c) : A_(a),B_(b),C_(c),isLine_(a!=0||b!=0)
{
	if(B_!=0)
	{
		isSlope_=true;
		k_=-A_/B_;
		b_=-C_/B_;
	}
	else
	{
		k_=0;
		b_=0;
		isSlope_=false;
	}
}
inline Line_::Line_(double k,double b) : k_(k),b_(b),A_(k),B_(-1),C_(b),isSlope_(true),isLine_(true) {}
template<typename _Tp> inline Line_::Line_(const DPoint_<_Tp> & pt1,const DPoint_<_Tp> & pt2)
{
	if(pt1!=pt2)
	{
		isLine_=true;
		if(pt1.x_!=pt2.x_) 
		{
			k_=double(pt2.y_-pt1.y_)/double(pt2.x_-pt1.x_);
			b_=double(pt1.y_*pt2.x_-pt2.y_*pt1.x_)/double(pt2.x_-pt1.x_);
			A_=k_;
			B_=-1.0;
			C_=b_;
			isSlope_=true;
		}
		else
		{
			A_=1;
			B_=0.0;
			C_=-pt1.x_;
			k_=0;
			b_=0;
			isSlope_=false;
		}
	}
	else
	{
		A_=0;B_=0;C_=0;k_=0;b_=0;
		isLine_=false;
		isSlope_=false;
	}
}
template<typename _Tp> inline  Line_::Line_(double k,const DPoint_<_Tp> & pt)
{
	k_ = k;
	b_ = double(pt.y_-k*pt.x_);
	A_ = k;
	B_ = -1.0;
	C_ = b_;
	isLine_=true;
	isSlope_=true;
}
inline bool Line_::isParallel(const Line_ & line) const
{
	if(isLine_&&line.isLine_)
		return((line.A_*B_-A_*line.B_)==0);
	else
		return (false);
}
inline bool Line_::isOrthogonal(const Line_ & line) const
{
	if(isLine_&&line.isLine_)
		return((A_*line.A_+B_*line.B_)==0);
	else
		return (false);
}

template<typename _Tp> inline double Line_::distance(const DPoint_<_Tp> & pt) const 
{
	return (std::abs(A_*pt.x_+B_*pt.y_+C_)/sqrt(A_*A_+B_*B_));
}
inline double Line_::distance(const Line_ & line) const
{
	if(isParallel(line))
		return (std::abs(C_-line.C_)/sqrt(A_*A_+B_*B_));
	else
		return (0.0);
}
inline DPoint_<double> Line_::crosspoint(const Line_ & line) const
{
	DPoint_<double> pt=DPoint_<double>(0.0,0.0);
	double ycoef=double(A_*line.B_-line.A_*B_);
	if(isLine_&&line.isLine_&&ycoef!=0)
	{
		pt.x_=double(B_*line.C_-line.B_*C_)/ycoef;
		pt.y_=double(-A_*line.C_+line.A_*C_)/ycoef;
	}
	return pt;
}
//!  get the  vertical line which passes through point pt;  
template<typename _Tp> static inline Line_ verticalline(const Line_ & line,const DPoint_<_Tp> & pt) 
{
	if(line.isLine_)
	{
		if(line.isSlope_)
		{
			if(line.k_==0)
				return Line_(1,0,-pt.x_);
			else
				return Line_(-1.0/line.k_,pt);
		}  
		else
			return Line_(0.0,pt.y_);
	}
	else
		return Line_();
}
//! get the point which has the dis with pt in line 
template<typename _Tp> static inline DPoint_<_Tp> pointinline(const Line_ & line , const DPoint_<_Tp> & pt,double dis)
{
	
	double theta=atan2(line.A_,-line.B_);
	return DPoint_<_Tp>(_Tp(pt.x_+dis*cos(theta)),_Tp(pt.y_+dis*sin(theta)));
}

//! get the vertical point which is in the line, and the new line connects the vertical point 
//! and input point pt is vertical with original line
template<typename _Tp> static inline DPoint_<_Tp> verticalpoint(const Line_ & line , const DPoint_<_Tp> & pt)
{
	DPoint_<double> outpt=DPoint_<double>(0.0,0.0);
	if(line.isLine_)
	{
		Line_ vertical_line=verticalline(line,pt);
		outpt=line.crosspoint(vertical_line);
	}
	return outpt;
}

}

#endif //!__LINE_HPP_
