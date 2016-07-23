#include "errortable.h"
using namespace nubot;

ErrorTable::ErrorTable()
{
    startx_ = field_info_.xline_[6]-100;
    endx_   = field_info_.xline_[0]+100;
    starty_ = field_info_.yline_[5]-100;
    endy_   = field_info_.yline_[0]+100;
    xlong_  = endx_-startx_+1;
    ylong_  = endy_-starty_+1;

    DistoMarkLine_=new double[xlong_*ylong_];
    Diff_X_=new double[xlong_*ylong_];
    Diff_Y_=new double[xlong_*ylong_];

    memset(DistoMarkLine_,0,xlong_*ylong_*sizeof(int));
    memset(Diff_X_,0,xlong_*ylong_*sizeof(double));
    memset(Diff_Y_,0,xlong_*ylong_*sizeof(double));
}

void
ErrorTable::getErrorTable()
{
   int xline_nums = field_info_.x_white_line_.size();
   int yline_nums = field_info_.y_white_line_.size();
   int count = 0;
   for(int j = starty_;j <=  endy_; j++)
   {
       for(int i = startx_; i <= endx_;i++)
       {
          double Mindis = DBL_MAX;
          DPoint pt(i,j);
          for(int m = 0 ;m <xline_nums ;m++ )
          {
             LineSegment & line_tmp = field_info_.x_white_line_[m];
             double distance_tmp = line_tmp.distance(pt);
             if(distance_tmp < Mindis)
                 Mindis = distance_tmp;
          }

          for(int n = 0 ;n <yline_nums ;n++ )
          {
             LineSegment & line_tmp = field_info_.y_white_line_[n];
             double distance_tmp = line_tmp.distance(pt);
             if(distance_tmp < Mindis)
                 Mindis = distance_tmp;
          }

          double dis_circle = field_info_.centercircle_.center_.distance(pt);
          dis_circle = std::abs(dis_circle - field_info_.centercircle_.radius_);
          if(dis_circle < Mindis)
              Mindis = dis_circle;

          if(field_info_.isInInterRect(pt))
          {
              int nums_circles = field_info_.postcircle_.size();
              for(int t =0 ; t< nums_circles; t++)
              {
                  Circle & postcircle = field_info_.postcircle_[t];
                  double postcircle_dis = postcircle.center_.distance(pt);
                  postcircle_dis = std::abs(postcircle_dis - postcircle.radius_);
                  if(postcircle_dis < Mindis)
                      Mindis = postcircle_dis;
              }
          }

          DistoMarkLine_[count]  = Mindis;
          count++;
       }
    }

}
void
ErrorTable::getDiffTable()
{
    for(int i=1;i<ylong_-1;i++)
        for(int j=1;j<xlong_-1;j++)
        {
            int lb=DistoMarkLine_[(i-1)*xlong_+j-1];
            int mb=DistoMarkLine_[(i-1)*xlong_+j];
            int rb=DistoMarkLine_[(i-1)*xlong_+j+1];
            int lm=DistoMarkLine_[i*xlong_+j-1];
            int rm=DistoMarkLine_[i*xlong_+j+1];
            int lu=DistoMarkLine_[(i+1)*xlong_+j-1];
            int mu=DistoMarkLine_[(i+1)*xlong_+j];
            int ru=DistoMarkLine_[(i+1)*xlong_+j+1];
            Diff_X_[i*xlong_+j]=(2*rm-2*lm+ru+rb-lu-lb)/8.0;
            Diff_Y_[i*xlong_+j]=(2*mu-2*mb+lu+ru-lb-rb)/8.0;
        }

    for(int i=1;i<xlong_-1;i++)
    {
        Diff_X_[i]=Diff_X_[xlong_+i];
        Diff_X_[(ylong_-1)*xlong_+i]=Diff_X_[(ylong_-1)*xlong_+i];
        Diff_Y_[i]=Diff_Y_[xlong_+i];
        Diff_Y_[(ylong_-1)*xlong_+i]=Diff_Y_[(ylong_-1)*xlong_+i];
    }
    for(int j=1;j<ylong_-1;j++)
    {
        Diff_X_[j*xlong_]=Diff_X_[j*xlong_+1];
        Diff_X_[j*xlong_+xlong_-1]=Diff_X_[j*xlong_+xlong_-2];
        Diff_Y_[j*xlong_]=Diff_Y_[j*xlong_+1];
        Diff_Y_[j*xlong_+xlong_-1]=Diff_Y_[j*xlong_+xlong_-2];
    }

    Diff_X_[0]=(Diff_X_[1]+Diff_X_[xlong_])/2.0;
    Diff_Y_[0]=(Diff_Y_[1]+Diff_Y_[xlong_])/2.0;
    Diff_X_[xlong_-1]=(Diff_X_[xlong_-2]+Diff_X_[2*xlong_-1])/2.0;
    Diff_Y_[xlong_-1]=(Diff_Y_[xlong_-2]+Diff_Y_[2*xlong_-1])/2.0;
    Diff_X_[(ylong_-1)*xlong_]=(Diff_X_[(ylong_-2)*xlong_]+Diff_X_[(ylong_-1)*xlong_+1])/2.0;
    Diff_Y_[(ylong_-1)*xlong_]=(Diff_Y_[(ylong_-2)*xlong_]+Diff_Y_[(ylong_-1)*xlong_+1])/2.0;
    Diff_X_[(ylong_-1)*xlong_+xlong_-1]=(Diff_X_[(ylong_-1)*xlong_+xlong_-2]+Diff_X_[(ylong_-2)*xlong_+xlong_-1])/2.0;
    Diff_Y_[(ylong_-1)*xlong_+xlong_-1]=(Diff_Y_[(ylong_-1)*xlong_+xlong_-2]+Diff_Y_[(ylong_-2)*xlong_+xlong_-1])/2.0;
}
