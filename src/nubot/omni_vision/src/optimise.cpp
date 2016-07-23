#include "nubot/omni_vision/optimise.h"
#include <fstream>
using namespace nubot;


Optimise::Optimise(std::string errorpath,std::string diffpathX,std::string diffpathY,Transfer & _transfercoor)
{
    startx_ =-fildinfo_.xline_[0]-100;
    endx_   = fildinfo_.xline_[0]+100;
    starty_ =-fildinfo_.yline_[0]-100;
    endy_   = fildinfo_.yline_[0]+100;
    xlong_  =endx_-startx_+1;
    ylong_  =endy_-starty_+1;

    DistoMarkLine=new double[xlong_*ylong_];
    Diff_X=new double[xlong_*ylong_];
    Diff_Y=new double[xlong_*ylong_];

    std::ifstream ErrorTable_Read(errorpath.c_str(), std::ios::binary|std::ios::in);
    ErrorTable_Read.read((char *)DistoMarkLine,sizeof(double)*xlong_*ylong_);
    ErrorTable_Read.close();

    std::ifstream DiffXTable_Read(diffpathX.c_str(), std::ios::binary|std::ios::in);
    DiffXTable_Read.read((char *)Diff_X,sizeof(double)*xlong_*ylong_);
    DiffXTable_Read.close();

    std::ifstream DiffYTable_Read(diffpathY.c_str(), std::ios::binary|std::ios::in);
    DiffYTable_Read.read((char *)Diff_Y,sizeof(double)*xlong_*ylong_);
    DiffYTable_Read.close();
}


double 
Optimise::caculateErrors(const vector<double>& weights,const vector<DPoint> & polar_pts,const DPoint & pt,const Angle & phi)
{   
    //std::vector<PPoint> pts;
    //transfer_->rotate(polar_pts,phi,pts);
    double cosphi = Angcos(phi);
    double sinphi = Angsin(phi);
    double templut=0;
    double err=0;
    double dist=0;
    double c2=250.0*250.0;

    size_t numtrans=polar_pts.size();
    for(size_t i=0;i<numtrans;i++)
    {
        DPoint relpts = DPoint(polar_pts[i].x_*cosphi-polar_pts[i].y_*sinphi,
                               polar_pts[i].x_*sinphi+polar_pts[i].y_*cosphi);
        DPoint worldpts=pt+relpts;
        if((std::abs((int)(worldpts.y_))<=endy_)&&(std::abs((int)(worldpts.x_))<=endx_))
            templut = DistoMarkLine[((int)(worldpts.y_)-starty_)*xlong_+(int)(worldpts.x_)-startx_];
        else
            templut=500;
        dist=(double)templut;
        double ef = c2+dist*dist;
        err+=weights[i]*(1-c2/ef);
    }
    return err;
}

void
Optimise::calculateGradient(const vector<double>& weights,const vector<DPoint> & polar_pts,
                            double& err, DPoint2d & grad, double& dphi, const DPoint & xy, const Angle & phi)
{
    // std::vector<PPoint> pts;
    //transfer_->rotate(polar_pts,phi,pts);
    double cosphi = Angcos(phi);
    double sinphi = Angsin(phi);
    double c2=250.0*250.0;
    err=0;
    double dist(0.0);
    double templut=0;
    grad=DPoint2d(0,0);
    dphi=0;
    DPoint2d ddistdpos(0.0,0.0);
    int numtrans=polar_pts.size();
    double dphi_tmp(0),err_tmp(0),grad_tmpx(0),grad_tmpy(0);
#ifdef using_openmp
#pragma omp parallel for reduction(+:dphi_tmp,err_tmp,grad_tmpx,grad_tmpy)
#endif
    for(int i=0;i<numtrans;i++)
    {
        DPoint relpts = DPoint(polar_pts[i].x_*cosphi-polar_pts[i].y_*sinphi,
                               polar_pts[i].x_*sinphi+polar_pts[i].y_*cosphi);
        DPoint worldpts=xy+relpts;
        if((std::abs((int)(worldpts.y_))<=endy_)&&(std::abs((int)(worldpts.x_))<=endx_))
            templut = DistoMarkLine[((int)(worldpts.y_)-starty_)*xlong_+(int)(worldpts.x_)-startx_];
        else
            templut=500;
        dist=(double)templut;
        double ef = c2+dist*dist;
        err_tmp+=weights[i]*(1-c2/ef);

        double derrddist = (2*c2*dist)/(ef*ef);
        ddistdpos=DPoint2d(0.0,0.0);
        if((std::abs((int)(worldpts.y_))<=endy_)&&(std::abs((int)(worldpts.x_))<=endx_))
        {
            ddistdpos.x_=Diff_X[((int)(worldpts.y_)-starty_)*xlong_+(int)(worldpts.x_)-startx_];
            ddistdpos.y_=Diff_Y[((int)(worldpts.y_)-starty_)*xlong_+(int)(worldpts.x_)-startx_];
        }
        grad_tmpx +=  weights[i]*derrddist*ddistdpos.x_,
                grad_tmpy +=  weights[i]*derrddist*ddistdpos.y_;
        dphi_tmp  += weights[i]*derrddist*(-ddistdpos.x_*relpts.y_+ddistdpos.y_*relpts.x_);
    }
    err  = err_tmp;
    dphi = dphi_tmp;
    grad = DPoint2d(grad_tmpx,grad_tmpy);
}

double 
Optimise::process(const vector<double>& weights,const vector<DPoint> &  whites_polar,DPoint& xy, Angle &ang)
{
    DPoint xy_tmp=xy;
    Angle  ang_tmp=ang;
    double param[3]={0};
    double best_param[3] ={0};
    double grad [3]={0};  // Gradient
    double stepwidth [3] = {4.0,4.0,0.1};
    static bool FirstOptimise=false;
    size_t niter(25);
    double min_error = 1e6;
    if(!FirstOptimise)
    {
        stepwidth[0] = 16.0;
        stepwidth[1] = 16.0;
        stepwidth[2] = 0.4;
        niter=50;
        FirstOptimise=true;
    }
    double latest_grad [3] = {0,0,0};
    double err=0.0;
    DPoint2d grad_pts(0,0);
    double  grad_ang(0);
    for (size_t i=0; i<niter; i++)
    {
        calculateGradient(weights,whites_polar,err,grad_pts,grad_ang, xy_tmp, ang_tmp);
        param[0] = double(xy_tmp.x_);
        param[1] = double(xy_tmp.y_);
        param[2] = ang_tmp.radian_;

        if(err < min_error)
        {
            best_param[0] = param[0];
            best_param[1] = param[1];
            best_param[2] = param[2];
            min_error = err;
        }
        grad[0] = grad_pts.x_;
        grad[1] = grad_pts.y_;
        grad[2] = grad_ang;

        Angle delta_angle =ang-param[2];
        err += 2.5*delta_angle.radian_*delta_angle.radian_;

        for (size_t  j=0; j<3; j++)
        {
            if (grad[j]==0)
                latest_grad[j]=0;
            else
            {
                if (grad[j]*latest_grad[j]>0)
                    stepwidth[j]*=1.2;
                else if (grad[j]*latest_grad[j]<0)
                    stepwidth[j]*=0.5;

                latest_grad[j]=grad[j];

                if (grad[j]>0)
                    param[j]-=stepwidth[j];
                else if (grad[j]<0)
                    param[j]+=stepwidth[j];
            }
        }
        xy_tmp =DPoint(param[0],param[1]);
        ang_tmp=Angle(param[2]);
    }
    xy =DPoint(best_param[0],best_param[1]);
    ang=Angle(best_param[2]);
    return err;
}

double
Optimise::analyse(const vector<double>& weights,const vector<DPoint> & polar_pts,DPoint2d & hxy,  double & hphi,const DPoint & xy, const Angle & phi)
{
    // std::vector<PPoint> pts;
    //transfer_->rotate(polar_pts,phi,pts);
    double cosphi = Angcos(phi);
    double sinphi = Angsin(phi);
    double err=0;
    hxy.x_=hxy.y_=0;
    hphi=0.0;
    double derr(0);
    double dderr(0);
    double dist(0);
    DPoint2d ddist(0.0,0.0);

    DPoint2d dposdphi(0.0,0.0);
    DPoint2d ddposdphi2(0.0,0.0);

    double c2=250.0*250.0;
    double c=250.0;

    int templut=0;
    size_t numtrans=polar_pts.size();
    double err_tmp(0),hphi_tmp(0),hxy_tmpx(0),hxy_tmpy(0);
#ifdef using_openmp
#pragma omp parallel for reduction(+:err_tmp,hphi_tmp,hxy_tmpx,hxy_tmpy)
#endif
    for(size_t i=0;i<numtrans;i++)
    {
        DPoint pt = DPoint(polar_pts[i].x_*cosphi-polar_pts[i].y_*sinphi,
                           polar_pts[i].x_*sinphi+polar_pts[i].y_*cosphi);
        DPoint pos=xy+pt;
        if((std::abs((int)(pos.y_))<=endy_)&&(std::abs((int)(pos.x_))<=endx_))
            templut = DistoMarkLine[((int)(pos.y_)-starty_)*xlong_+(int)(pos.x_)-startx_];
        else
            templut=500;
        dist=(double)templut;
        ddist=DPoint2d(0.0,0.0);
        err_tmp +=weights[i]*(1-c2/(c2+dist*dist));

        if (dist<2*c)
        {
            derr=dist/c2;
            dderr=1/c2;
            if((std::abs((int)(pos.y_))<=endy_)&&(std::abs((int)(pos.x_))<=endx_))
            {
                ddist.x_=Diff_X[((int)(pos.y_)-starty_)*xlong_+(int)(pos.x_)-startx_];
                ddist.y_=Diff_Y[((int)(pos.y_)-starty_)*xlong_+(int)(pos.x_)-startx_];
            }
            dposdphi.x_ =  -pt.y_ ;
            dposdphi.y_ =   pt.x_;
            ddposdphi2.x_ =-pt.x_;
            ddposdphi2.y_ =-pt.y_;
            hxy_tmpx += weights[i]*dderr*ddist.x_*ddist.x_;
            hxy_tmpy += weights[i]*dderr*ddist.y_*ddist.y_;
            double t1 = ddist.x_*dposdphi.x_+ddist.y_*dposdphi.y_;
            double t2 = ddist.x_*ddposdphi2.x_+ddist.y_*ddposdphi2.y_;
            hphi_tmp += weights[i]*(dderr*t1*t1+derr*t2);
        }
    }
    err  = err_tmp;
    hphi = hphi_tmp;
    hxy  = DPoint2d(hxy_tmpx,hxy_tmpy);
    return err;
}
