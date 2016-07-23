#include "calibration_dialog.h"
#include "ui_calibration_dialog.h"
#include "opencv2/opencv.hpp"
#include "errortable.h"
#include <fstream>
#include <sstream>
Dialog::Dialog(image_subscribe & _image,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    image_sub_ = &_image;

    start_pts_.push_back(QPoint(0,0));
    start_pts_.push_back(QPoint(520,0));
    start_pts_.push_back(QPoint(520,0));

    scale_factor_=2;

    //the drawPolygon is in the center 0-511
    poly_pts_.push_back(QPoint(256*scale_factor_/2-30,256*scale_factor_/2-30));
    poly_pts_.push_back(QPoint(256*scale_factor_/2+30,256*scale_factor_/2-30));
    poly_pts_.push_back(QPoint(256*scale_factor_/2+30,256*scale_factor_/2+30));
    poly_pts_.push_back(QPoint(256*scale_factor_/2-30,256*scale_factor_/2+30));

    /* @breif the margal point */
    Line.push_back(QPoint(0,256*scale_factor_+15));
    Line.push_back(QPoint(255*scale_factor_,256*scale_factor_+15));
    Line_pts_.push_back(QPoint(0,256*scale_factor_+15));
    Line_pts_.push_back(QPoint(255*scale_factor_,256*scale_factor_+15));


    segment_img_.resize(3);
    orignal_img_.resize(3);
    image.resize(3);
    bufferImg_.resize(3);
    templeImg_.resize(3);
    for(std::size_t i =0 ; i < 3; i++)
    {
        is_screen_update_.push_back(false);
        is_draw_Img_.push_back(false);
        segment_img_[i].create(cv::Size(256,256),CV_8UC3);
    }
    for(std::size_t i =0 ; i < 3; i++)
    {
       calib_interim_result_.push_back(poly_pts_);
       calib_final_result_.push_back(poly_pts_);
       calib_Line_inter_result_.push_back(Line_pts_);
       calib_Line_final_result_.push_back(Line_pts_);
    }
    calib_color.push_back(2);
    calib_color.push_back(2);
    calib_color.push_back(1);

    Img_selected_=0;
    nearest_pt_label_=-1;
    nearest_line_pt_=-1;
    nearest_diameter_pt_=-1;
    circle_diameter_.push_back(QPoint(50,200));
    circle_diameter_.push_back(QPoint(150,200));
    is_draw_big_circle_=false;
    is_draw_small_circle_=false;
    circle_radius_=500;
    small_radius_=0;



    color_space_img_.resize(3);
    color_space_img_[0].create(1,64*64*64,CV_8UC3);
    color_space_img_[1].create(1,64*64*64,CV_8UC3);
    color_space_img_[2].create(1,64*64*64,CV_8UC3);
    int idx=0;
    for(std::size_t y =0 ;y < 64 ;y++)
        for(std::size_t u=0 ;u < 64 ;u++)
             for(std::size_t v=0 ;v < 64 ;v++)
             {
                 color_space_img_[2].at<cv::Vec3b>(0,idx)=cv::Vec3b(4*y,4*u,4*v); //y u v;
                 CTable_[idx]=3;
                 idx++;
              }
    cv::cvtColor(color_space_img_[2],color_space_img_[0],CV_YUV2RGB);
    cv::cvtColor(color_space_img_[0],color_space_img_[1],CV_RGB2HSV);


    color_space_bgr_.resize(3);
    color_space_bgr_[0].create(1,64*64*64,CV_8UC3);
    color_space_bgr_[1].create(1,64*64*64,CV_8UC3);
    color_space_bgr_[2].create(1,64*64*64,CV_8UC3);
    idx=0;
    for(std::size_t b =0 ;b< 64 ;b++)
        for(std::size_t g=0 ;g < 64 ;g++)
             for(std::size_t r=0 ;r < 64 ;r++)
             {
                 color_space_bgr_[0].at<cv::Vec3b>(0,idx)=cv::Vec3b(4*b,4*g,4*r); //b,g,r;
                 CTable_bgr_[idx]=3;
                 idx++;
              }
    cv::cvtColor(color_space_bgr_[0],color_space_bgr_[1],CV_BGR2HSV);
    cv::cvtColor(color_space_bgr_[0],color_space_bgr_[2],CV_BGR2YUV);



    show_poly_segment_=QPoint(256*scale_factor_/2,256*scale_factor_/2);
    show_line_segment_=QPoint(255*scale_factor_/2,256*scale_factor_+15);
    setMouseTracking(true);
    is_read_whites_=false;

    char * environment;
    int agent;
    if((environment = getenv("AGENT"))==NULL)
    {
        ROS_ERROR("this agent number is not read by robot");
        return ;
    }
    agent = atoi(environment);
    std::stringstream ss;
    ss<<agent;
    calibration_result_dir_="/home/nubot"+ss.str()+"/nubot_ws/src/nubot/omni_vision/calib_results/"+ss.str();
    ui->setupUi(this);
}

Dialog::~Dialog()
{

    delete ui;
}

void
Dialog::on_receive_image_btn_clicked()
{
    image_sub_->set_update_state(true);
    bool update_img=true;
    while(update_img)
    {
        if(!image_sub_->get_update_state())
        {
            cv::cvtColor(image_sub_->get_camera_img(),orignal_img_[0],CV_BGR2RGB);
            update_img=false;
        }
    }
    cv::cvtColor(orignal_img_[0],orignal_img_[2],CV_RGB2YUV);
    cv::cvtColor(orignal_img_[0],orignal_img_[1],CV_RGB2HSV);

    image[0]=QImage((const unsigned char*)(orignal_img_[0].data),
                     orignal_img_[0].cols,orignal_img_[0].rows,
                     orignal_img_[0].cols*orignal_img_[0].channels(),
                     QImage::Format_RGB888);
    segment_img_[0]=orignal_img_[0];
    is_screen_update_[0]=true;
    is_draw_Img_[0]=false;
    paint(image[0]);
    bufferImg_[0]=image[0];
    Img_selected_=0;
}

void
Dialog::on_loadImage_Btn_clicked()
{
    std::string str_tmp =  calibration_result_dir_+ "/";
    QString initial_path=QString::fromStdString(str_tmp);
    QFileDialog FileDialog;
    QString info=FileDialog.getOpenFileName(this,tr("Open　Image"),initial_path,
                                           tr("Image　Files(*.jpg *.bmp *.png)"));

    calibration_result_dir_=info.toStdString();
    if(!info.isEmpty())
    {
        std::string file_path=info.toStdString();
        cv::Mat img=cv::imread(file_path);
        cv::cvtColor(img,orignal_img_[0],CV_BGR2RGB);
        cv::cvtColor(orignal_img_[0],orignal_img_[2],CV_RGB2YUV);
        cv::cvtColor(orignal_img_[0],orignal_img_[1],CV_RGB2HSV);
        image[0]=QImage((const unsigned char*)(orignal_img_[0].data),
                         orignal_img_[0].cols,orignal_img_[0].rows,
                         orignal_img_[0].cols*orignal_img_[0].channels(),
                         QImage::Format_RGB888);
        segment_img_[0]=orignal_img_[0];
        is_screen_update_[0]=true;
        is_draw_Img_[0]=false;
        paint(image[0]);
        bufferImg_[0]=image[0];
        Img_selected_=0;
    }
}

void Dialog::on_HSI_RadioBtn_clicked()
{
    if(orignal_img_[1].empty())
        return;
    segment_img_[1].setTo(cv::Scalar(255,255,255));
    for(std::size_t i =0 ; i < orignal_img_[1].rows;i++ )
    {
        for(std::size_t j= 0 ; j < orignal_img_[1].cols; j++)
        {
            cv::Vec3b color_hsi,color_rgb;
            color_hsi=orignal_img_[1].at<cv::Vec3b>(i,j);
            color_rgb=orignal_img_[0].at<cv::Vec3b>(i,j);
            segment_img_[1].at<cv::Vec3b>(255-int(color_hsi[1]),int(color_hsi[0]))=color_rgb;
        }
    }
    cv::Mat show_hsi;
    cv::resize(segment_img_[1],show_hsi, cv::Size(0,0), scale_factor_, scale_factor_, CV_INTER_NN);

    image[1]=QImage((const unsigned char*)(show_hsi.data),
                              show_hsi.cols,show_hsi.rows,
                              show_hsi.cols*show_hsi.channels(),
                              QImage::Format_RGB888);


    is_draw_Img_[1]=false;
    is_screen_update_[1]=true;
    is_screen_update_[2]=false;
    paint(image[1]);
    bufferImg_[1]=image[1];
    Img_selected_=1;
}

void Dialog::on_YUV_RadioBtn_clicked()
{
    if(orignal_img_[2].empty())
        return;
    segment_img_[2].setTo(cv::Scalar(255,255,255));
    for(std::size_t i =0 ; i < orignal_img_[2].rows; i++ )
    {
        for(std::size_t j= 0 ; j < orignal_img_[2].cols; j++)
        {
            cv::Vec3b color_yuv,color_rgb;
            color_yuv=orignal_img_[2].at<cv::Vec3b>(i,j);
            color_rgb=orignal_img_[0].at<cv::Vec3b>(i,j);
            segment_img_[2].at<cv::Vec3b>(int(color_yuv[2]),int(255-color_yuv[1]))=color_rgb;
        }
    }
    cv::Mat show_yuv;
    cv::resize(segment_img_[2],show_yuv, cv::Size(0,0), scale_factor_, scale_factor_,CV_INTER_NN);
    image[2]=QImage((const unsigned char*)(show_yuv.data),
                              show_yuv.cols,show_yuv.rows,
                              show_yuv.cols*show_yuv.channels(),
                              QImage::Format_RGB888);
    is_draw_Img_[2]=false;
    is_screen_update_[2]=true;
    is_screen_update_[1]=false;
    paint(image[2]);
    bufferImg_[2]=image[2];
    Img_selected_=2;
}

int
Dialog::select_nearest_pt(const QVector<QPoint> & pts,QPoint pt,QPoint start_pt)
{
    double Min_dis=1000000000;
    int nearest=0;
    for(std::size_t i =0 ;i < pts.size() ; i++ )
    {
       QPoint tmp=pt-start_pt-pts[i];
       double dis=tmp.x()*tmp.x()+tmp.y()*tmp.y();
       if(Min_dis>dis)
       {
           Min_dis=dis;
           nearest=i;
       }
    }
    return nearest;
}

void Dialog::on_ball_seg_Btn_clicked()
{
    if(Img_selected_==0)
        return ;
     poly_pts_ = calib_interim_result_[0];
     Line_pts_ = calib_Line_inter_result_[0];
     bufferImg_[Img_selected_]=image[Img_selected_];
     is_draw_Img_[Img_selected_]=true;
     paint(bufferImg_[Img_selected_]);
     is_draw_Img_[Img_selected_]=false;
}


void Dialog::on_obs_seg_Btn_clicked()
{
      if(Img_selected_==0)
          return ;
      poly_pts_= calib_interim_result_[1];
      Line_pts_=calib_Line_inter_result_[1];
      bufferImg_[Img_selected_]=image[Img_selected_];
      is_draw_Img_[Img_selected_]=true;
      paint(bufferImg_[Img_selected_]);
      is_draw_Img_[Img_selected_]=false;
}

void Dialog::on_field_segment_Btn_clicked()
{
    if(Img_selected_==0)
        return ;
    poly_pts_= calib_interim_result_[2];
    Line_pts_=calib_Line_inter_result_[2];
    bufferImg_[Img_selected_]=image[Img_selected_];
    is_draw_Img_[Img_selected_]=true;
    paint(bufferImg_[Img_selected_]);
    is_draw_Img_[Img_selected_]=false;
}
void
Dialog::mousePressEvent(QMouseEvent * event)
{
    bufferImg_[Img_selected_]=image[Img_selected_];
    QPoint & pt=start_pts_[Img_selected_];
    QPoint pttmp=QPoint(event->x(),event->y());
    double obsdist=std::sqrt((pttmp.x()-circle_center_.x())*(pttmp.x()-circle_center_.x())+
               (pttmp.y()-circle_center_.y())*(pttmp.y()-circle_center_.y()));
    double r = 1.736844*tan(0.0070288*obsdist); /*get meter distance*/

    /* @brief this is not HSI or YUC Image */
    if(Img_selected_!=0)
    {
        if(event->x() <pt.x()+scale_factor_*256 && event->x()>pt.x())
        {
           if((is_screen_update_[1]||is_screen_update_[2]))
           {
               // calibration the uv or hs color
               if(event->y()<pt.y()+scale_factor_*256 && event->y()>pt.y())
                {
                   is_draw_Img_[Img_selected_]=true;
                   nearest_pt_label_=select_nearest_pt(poly_pts_,QPoint(event->x(),event->y()),pt);
                   poly_pts_[nearest_pt_label_]=QPoint(event->x(),event->y())-pt;
               }
               // calibration the y or i
               if(event->y()< Line[1].y()+7 && event->y()>Line[1].y()-7)
               {
                    is_draw_Img_[Img_selected_]=true;
                    nearest_line_pt_=select_nearest_pt(Line_pts_,QPoint(event->x(),event->y()),pt);
                    Line_pts_[nearest_line_pt_]=QPoint(event->x(),Line[1].y())-pt;
               }
           }
        }
    }
    /*! @brief draw the circle */
    if(Img_selected_==0)
    {
        if(event->x()<orignal_img_[Img_selected_].cols && event->x()>0 &&
           event->y()<orignal_img_[Img_selected_].rows && event->y()>0)
        {
            if(is_draw_big_circle_)
            {
              is_draw_Img_[Img_selected_]=true;
              nearest_diameter_pt_=select_nearest_pt(circle_diameter_,QPoint(event->x(),event->y()),pt);
              circle_diameter_[nearest_diameter_pt_]=QPoint(event->x(),event->y());
            }
            if(is_draw_small_circle_)
            {
               is_draw_Img_[Img_selected_]=true;
               small_edge_=QPoint(circle_center_.x()-10,circle_center_.y());
            }
        }
    }
}

void
Dialog::mouseMoveEvent(QMouseEvent * event)
{
    QPoint & pt=start_pts_[Img_selected_];
    if(Img_selected_!=0)
    {
        if(event->x() <pt.x()+scale_factor_*256 && event->x()>pt.x())
        {
           if((is_screen_update_[1]||is_screen_update_[2])&&is_draw_Img_[Img_selected_])
           {

               if(event->y()<pt.y()+scale_factor_*256 && event->y()>pt.y())
               {
                 poly_pts_[nearest_pt_label_]=QPoint(event->x(),event->y())-pt;
                 templeImg_[Img_selected_] = bufferImg_[Img_selected_];
                 paint(templeImg_[Img_selected_]);
               }
               if(event->y()< Line[1].y()+7 && event->y()>Line[1].y()-7 )
               {
                   Line_pts_[nearest_line_pt_]=QPoint(event->x(),Line[1].y())-pt;
                   templeImg_[Img_selected_] = bufferImg_[Img_selected_];
                   paint(templeImg_[Img_selected_]);
               }
           }
        }
        /* the point in the orignal image*/
        if(event->x()<orignal_img_[0].cols && event->x()>0 &&
           event->y()<orignal_img_[0].rows && event->y()>0)
        {
             cv::Point pt(event->x(),event->y());
             cv::Vec3b color=orignal_img_[Img_selected_].at<cv::Vec3b>(pt);
             if(Img_selected_==1)
             {
                 show_poly_segment_=QPoint(color[0]*scale_factor_,(255-color[1])*scale_factor_);
                 show_line_segment_=QPoint(color[2]*scale_factor_,256*scale_factor_+15);
             }
             else
             {
                 show_poly_segment_=QPoint((255-color[1])*scale_factor_,color[2]*scale_factor_);
                 show_line_segment_=QPoint(color[0]*scale_factor_,256*scale_factor_+15);
             }
             templeImg_[Img_selected_] = bufferImg_[Img_selected_];
             paint(templeImg_[Img_selected_]);
        }

    }
    /*! @brief draw the circle */
    if(Img_selected_==0 && is_draw_Img_[Img_selected_])
    {
        if(event->x()<orignal_img_[Img_selected_].cols && event->x()>0 &&
           event->y()<orignal_img_[Img_selected_].rows && event->y()>0)
        {
            if(is_draw_big_circle_)
            {
                is_draw_Img_[Img_selected_]=true;
                circle_diameter_[nearest_diameter_pt_]=QPoint(event->x(),event->y());
                templeImg_[Img_selected_] = bufferImg_[Img_selected_];
                paint(templeImg_[Img_selected_]);
            }
            if(is_draw_small_circle_)
            {
               is_draw_Img_[Img_selected_]=true;
               small_edge_=QPoint(event->x(),event->y());
               templeImg_[Img_selected_] = bufferImg_[Img_selected_];
               paint(templeImg_[Img_selected_]);
            }
        }
    }
}

void
Dialog::mouseReleaseEvent(QMouseEvent * event)
{

    if(is_draw_Img_[Img_selected_])
    {
       bufferImg_[Img_selected_]=image[Img_selected_];
       update_calib_interim_result();
       paint(bufferImg_[Img_selected_]);
       is_draw_Img_[Img_selected_]=false;
    }
}
void
Dialog::update_calib_interim_result()
{

    if(ui->ball_seg_Btn->isChecked())
    {
       calib_interim_result_[0]=poly_pts_;
       calib_Line_inter_result_[0]=Line_pts_;
       calib_color[0]=Img_selected_;
    }
    if(ui->obs_seg_Btn->isChecked())
    {
       calib_interim_result_[1]=poly_pts_;
       calib_Line_inter_result_[1]=Line_pts_;
       calib_color[1]=Img_selected_;
    }
    if(ui->field_segment_Btn->isChecked())
    {
       calib_interim_result_[2]=poly_pts_;
       calib_Line_inter_result_[2]=Line_pts_;
       calib_color[2]=Img_selected_;
    }
}

/*! @brief get the orignal polygon  */
void
Dialog::update_calibration_result()
{
    for(std::size_t i =0 ;i < calib_interim_result_.size();i++)
    {
        QVector<QPoint> pts_tmp;
        QVector<QPoint> & pts = calib_interim_result_[i];
        for(std::size_t i=0; i < pts.size() ; i++)
        {
            QPoint pt= QPoint(pts[i].x()/scale_factor_,int(pts[i].y()/scale_factor_));
            pts_tmp.push_back(pt);
        }
       calib_final_result_[i]=pts_tmp;

       QVector<QPoint> ps_line_tmp;
       QVector<QPoint> & pts_line = calib_Line_inter_result_[i];
       for(std::size_t i=0; i < pts_line.size() ; i++)
       {
           QPoint pt= QPoint(pts_line[i].x()/scale_factor_,0);
           ps_line_tmp.push_back(pt);
       }
       calib_Line_final_result_[i]=ps_line_tmp;
    }
}

/*! @brief  only draw the shape in the image*/
void
Dialog::paint(QImage & img)
{
   QPainter painter(&img);
   painter.setPen(QPen(Qt::red,1,Qt::SolidLine));
   if(is_draw_Img_[Img_selected_])
   {
      if(Img_selected_!=0)
         painter.drawPolygon(QPolygon(poly_pts_));
      else
      {
          if(is_draw_big_circle_)
          {
             circle_center_=(circle_diameter_[0]+circle_diameter_[1])/2;
             QPoint vect=circle_diameter_[0]-circle_diameter_[1];
             circle_radius_=(int)std::sqrt(vect.x()*vect.x()+vect.y()*vect.y())/2.0;
             painter.drawEllipse(circle_center_,circle_radius_,circle_radius_);
          }
          if(is_draw_small_circle_)
          {
             QPoint vect=circle_center_-small_edge_;
             small_radius_=(int)std::sqrt(vect.x()*vect.x()+vect.y()*vect.y());
             painter.drawEllipse(circle_center_,small_radius_,small_radius_);
          }
      }
   }
   update();
}

void
Dialog::paintEvent(QPaintEvent * event)
{
    QPainter painter(this);
    for(std::size_t i =0 ;i <3 ;i++)
    {
      if(is_screen_update_[i])
      {
         if(!is_draw_Img_[i])
            painter.drawImage(start_pts_[i],bufferImg_[i]);
         else
            painter.drawImage(start_pts_[i],templeImg_[i]);
      }
    }
    if(Img_selected_!=0)
    {
       painter.setPen(QPen(Qt::blue,4,Qt::SolidLine));
       painter.drawLine(Line[0]+start_pts_[Img_selected_],Line[1]+start_pts_[Img_selected_]);
       painter.setPen(QPen(Qt::red,4,Qt::SolidLine));
       painter.drawEllipse(Line_pts_[0]+start_pts_[Img_selected_],2,2);
       painter.drawEllipse(Line_pts_[1]+start_pts_[Img_selected_],2,2);

       painter.setPen(QPen(QColor( 0, 255, 255),4,Qt::SolidLine));
       painter.drawEllipse(show_poly_segment_+start_pts_[Img_selected_],2,2);
       painter.drawEllipse(show_line_segment_+start_pts_[Img_selected_],2,2);
    }
    else
    {
        painter.setPen(QPen(Qt::red,4,Qt::SolidLine));
        if(is_draw_big_circle_)
        {
            painter.drawEllipse(circle_diameter_[0],2,2);
            painter.drawEllipse(circle_diameter_[1],2,2);
            painter.drawEllipse(circle_center_,2,2);
        }
        if(is_draw_small_circle_)
              painter.drawEllipse(circle_center_,2,2);
    }

}

/*! @brief show the segment result */
void
Dialog::on_segment_result_btn_clicked()
{
    update_calibration_result();
   /*! poly_pts_  is the color segment threa*/
    cv::Mat image_tmp=orignal_img_[0].clone();

    QVector< QPoint> pts;
    QVector<QPoint> third_color;
    pts.resize(3);
    third_color.resize(3);

    std::vector<cv::Vec3b> show_color;
    show_color.push_back(cv::Vec3b(255,69,0));
    show_color.push_back(cv::Vec3b(138,43,226));
    show_color.push_back(cv::Vec3b(0,100,0));

    for(std::size_t i =0 ; i < orignal_img_[Img_selected_].rows; i++ )
    {
        for(std::size_t j= 0 ; j < orignal_img_[Img_selected_].cols; j++)
        {
              cv::Vec3b color_rgb=orignal_img_[0].at<cv::Vec3b>(i,j);
              cv::Vec3b color_hsi=orignal_img_[1].at<cv::Vec3b>(i,j);
              cv::Vec3b color_yuv=orignal_img_[2].at<cv::Vec3b>(i,j);
              double dis_center=std::sqrt((circle_center_.x()-j)*(circle_center_.x()-j)+
                                   (circle_center_.y()-i)*(circle_center_.y()-i));
              if(dis_center<circle_radius_)
              {
                  pts[2]=QPoint(255-color_yuv[1],color_yuv[2]); //U V
                  pts[1]=QPoint(color_hsi[0],255-color_hsi[1]); //H,S
                  third_color[2]=QPoint(color_yuv[0],0);    //Y
                  third_color[1]=QPoint(color_hsi[2],0);    //I  calib_color[2] = 1;calib_color[1]=calib_color[0]=2
                  for(int k = 2; k >=0 ;k--)
                  {
                      bool is_inpoly = pnpoly(calib_final_result_[k],pts[calib_color[k]]);
                      bool is_line   = pnline(calib_Line_final_result_[k],third_color[calib_color[k]]);
                      if(is_inpoly&&is_line)
                         image_tmp.at<cv::Vec3b>(i,j)=show_color[k];
                  }
              }
        }
    }
    image[0] = QImage((const unsigned char*)(image_tmp.data),
                     image_tmp.cols,image_tmp.rows,
                     image_tmp.cols*image_tmp.channels(),
                     QImage::Format_RGB888);
    bufferImg_[0] = image[0];
    paint(bufferImg_[0]);
    update_yuv_table();
    update_bgr_table();
}
void Dialog::on_show_segmentBtn_clicked()
{
    cv::Mat image_tmp=orignal_img_[0].clone();

    QVector< QPoint> pts;
    QVector<QPoint> third_color;
    pts.resize(3);
    third_color.resize(3);

    std::vector<cv::Vec3b> show_color;
    show_color.push_back(cv::Vec3b(255,69,0));
    show_color.push_back(cv::Vec3b(138,43,226));
    show_color.push_back(cv::Vec3b(0,100,0));

    for(std::size_t i =0 ; i < orignal_img_[Img_selected_].rows; i++ )
    {
        for(std::size_t j= 0 ; j < orignal_img_[Img_selected_].cols; j++)
        {
              cv::Vec3b color_yuv=orignal_img_[2].at<cv::Vec3b>(i,j);
              double dis_center=std::sqrt((circle_center_.x()-j)*(circle_center_.x()-j)+
                                   (circle_center_.y()-i)*(circle_center_.y()-i));
              if(dis_center<circle_radius_)
              {
                  int k = CTable_[color_yuv[0]/4*64*64+color_yuv[1]/4*64+color_yuv[2]/4];
                  if(k<3)
                      image_tmp.at<cv::Vec3b>(i,j)=show_color[k];
              }
        }
    }
    image[0] = QImage((const unsigned char*)(image_tmp.data),
                     image_tmp.cols,image_tmp.rows,
                     image_tmp.cols*image_tmp.channels(),
                     QImage::Format_RGB888);
    bufferImg_[0] = image[0];
    paint(bufferImg_[0]);
}
void
Dialog::update_yuv_table()
{
    QVector< QPoint> pts;
    QVector<QPoint> third_color;
    pts.resize(3);
    third_color.resize(3);

    int idx=0;
    for(std::size_t y =0 ;y < 64 ;y++)
        for(std::size_t u=0 ;u < 64 ;u++)
             for(std::size_t v=0 ;v < 64 ;v++)
             {
                 cv::Vec3b color_rgb=color_space_img_[0].at<cv::Vec3b>(0,idx);
                 cv::Vec3b color_hsi=color_space_img_[1].at<cv::Vec3b>(0,idx);
                 cv::Vec3b color_yuv=color_space_img_[2].at<cv::Vec3b>(0,idx);

                 pts[2]=QPoint(255-color_yuv[1],color_yuv[2]); //U V
                 pts[1]=QPoint(color_hsi[0],255-color_hsi[1]); //H,S
                 third_color[2]=QPoint(color_yuv[0],0);    //Y
                 third_color[1]=QPoint(color_hsi[2],0);
                 CTable_[idx]=3;
                 for(int k = 2 ;k >=0 ;k--)
                 {
                     bool is_inpoly = pnpoly(calib_final_result_[k],pts[calib_color[k]]);
                     bool is_line   = pnline(calib_Line_final_result_[k],third_color[calib_color[k]]);
                     if(is_inpoly&&is_line)
                         CTable_[idx]=k;
                 }
                 idx++;
              }
}
void Dialog::update_bgr_table()
{
    QVector< QPoint> pts;
    QVector<QPoint> third_color;
    pts.resize(3);
    third_color.resize(3);

    int idx=0;
    for(std::size_t b =0 ;b < 64 ;b++)
        for(std::size_t g=0 ;g < 64 ;g++)
             for(std::size_t r=0 ;r < 64 ;r++)
             {
                 cv::Vec3b color_bgr=color_space_bgr_[0].at<cv::Vec3b>(0,idx);
                 cv::Vec3b color_hsi=color_space_bgr_[1].at<cv::Vec3b>(0,idx);
                 cv::Vec3b color_yuv=color_space_bgr_[2].at<cv::Vec3b>(0,idx);

                 pts[2]=QPoint(255-color_yuv[1],color_yuv[2]); //U V
                 pts[1]=QPoint(color_hsi[0],255-color_hsi[1]); //H,S
                 third_color[2]=QPoint(color_yuv[0],0);    //Y
                 third_color[1]=QPoint(color_hsi[2],0);

                 CTable_bgr_[idx]=3;
                 for(int k = 2 ;k >=0 ;k--)
                 {
                     bool is_inpoly = pnpoly(calib_final_result_[k],pts[calib_color[k]]);
                     bool is_line   = pnline(calib_Line_final_result_[k],third_color[calib_color[k]]);
                     if(is_inpoly&&is_line)
                         CTable_bgr_[idx]=k;
                 }
                 idx++;
              }
}

bool
Dialog::pnline(const QVector<QPoint> &pts, QPoint test_pt)
{
  if(pts.size()!=2)
      return false ;
  if((pts[0].x()-test_pt.x())*(pts[1].x()-test_pt.x())>0)
      return false;
  return true;
}


/*! @brief decide the test_pt whether it is located in ploygon ( pts )*/
bool
Dialog::pnpoly (const QVector<QPoint> & pts, QPoint test_pt)
{
    int nvert=pts.size();
    int minX(100000),maxX((-100000)),maxY(-100000),minY((100000));
    for(std::size_t i = 0; i <nvert ;i++)
    {
        if(pts[i].x()<minX)
            minX=pts[i].x();
        if(pts[i].x()>maxX)
            maxX=pts[i].x();
        if(pts[i].y()<minY)
            minY=pts[i].y();
        if(pts[i].y()>maxY)
            maxY=pts[i].y();
    }

    if (test_pt.x() < minX || test_pt.x() > maxX || test_pt.y() < minY || test_pt.y() > maxY)
        return false;

    int i, j;
    bool c = false;
    for (i = 0, j = nvert-1; i < nvert; j = i++)
    {
        if ( ( (pts[i].y()>test_pt.y()) != (pts[j].y()>test_pt.y()) ) &&
               (test_pt.x() < (pts[j].x()-pts[i].x()) * (test_pt.y()-pts[i].y())/double((pts[j].y()-pts[i].y()))+ pts[i].x()) )
            c = !c;
     }
    return c;
}


void Dialog::on_BigCircle_Btn_clicked()
{
    Img_selected_=0;
    is_draw_big_circle_=true;
    is_draw_Img_[Img_selected_]=true;
    is_draw_small_circle_=false;
    paint(bufferImg_[Img_selected_]);
    is_draw_Img_[Img_selected_]=false;
}

void Dialog::on_SmallCircle_Btn_clicked()
{
    //! the big circle parameter;
    circle_center_=(circle_diameter_[0]+circle_diameter_[1])/2;
    QPoint vect=circle_diameter_[0]-circle_diameter_[1];
    circle_radius_=(int)std::sqrt(vect.x()*vect.x()+vect.y()*vect.y())/2.0;

    Img_selected_=0;
    is_draw_small_circle_=true;
    is_draw_big_circle_=false;
    is_draw_Img_[Img_selected_]=true;
    paint(bufferImg_[Img_selected_]);
    is_draw_Img_[Img_selected_]=false;

}

void Dialog::on_Save_Result_Btn_clicked()
{
    std::string file_path=calibration_result_dir_+"/calibration.xml";
    cv::FileStorage calibration(file_path, cv::FileStorage::WRITE);
    calibration<<"Color_Calib"<<"[";
    for(std::size_t i =0 ; i < 3; i++)
    {
        calibration<<"{";
        calibration<<"poly_pts"<<"[";
        QVector<QPoint> & pts= calib_final_result_[i];
        for(std::size_t j =0; j< pts.size();j++)
            calibration<<pts[j].x()<<pts[j].y();
       calibration << "]";
       QVector<QPoint> & pts_line = calib_Line_final_result_[i];
       calibration<<"line_pts"<<"[";
       calibration<<pts_line[0].x()<<pts_line[1].x();
       calibration << "]"<<"}" ;
    }
    calibration.release();

    file_path=calibration_result_dir_+"/CTable.dat";
    std::ofstream CTable_Write(file_path.c_str(), std::ios::binary|std::ios::out);
    CTable_Write.write((char *)CTable_,sizeof(unsigned char)*64*64*64);
    CTable_Write.close();

    file_path=calibration_result_dir_+"/CTableBGR.dat";
    std::ofstream CTableBGR_Write(file_path.c_str(), std::ios::binary|std::ios::out);
    CTableBGR_Write.write((char *)CTable_bgr_,sizeof(unsigned char)*64*64*64);
    CTableBGR_Write.close();
}

void Dialog::on_Load_Result_Btn_clicked()
{
    std::string str_tmp =  calibration_result_dir_+ "/";
    QString initial_path=QString::fromStdString(str_tmp);
    QFileDialog FileDialog;
    QString path=FileDialog.getExistingDirectory(this, tr("Open Calibration Files"),
                                    initial_path,
                                    QFileDialog::ShowDirsOnly|QFileDialog::DontResolveSymlinks);

    calibration_result_dir_=path.toStdString();
    std::string file_path=calibration_result_dir_+"/CTable.dat";
    std::ifstream CTable_Read(file_path.c_str(), std::ios::binary|std::ios::in);
    CTable_Read.read((char *)CTable_,sizeof(unsigned char)*64*64*64);
    CTable_Read.close();

    file_path=calibration_result_dir_+"/CTableBGR.dat";
    std::ifstream CTableBGR_Read(file_path.c_str(), std::ios::binary|std::ios::out);
    CTableBGR_Read.read((char *)CTable_bgr_,sizeof(unsigned char)*64*64*64);
    CTableBGR_Read.close();


    file_path=calibration_result_dir_+"/ROI.xml";
    cv::FileStorage roi_file(file_path, cv::FileStorage::READ);
    int tmp1,tmp2;
    roi_file["center_point_row"]>>tmp2;
    roi_file["center_point_column"]>>tmp1;
    circle_center_=QPoint(tmp1,tmp2);
    roi_file["big_radius"] >>circle_radius_;
    roi_file ["small_radius"] >> small_radius_;
    roi_file ["rows"]>>height_;
    roi_file ["cols"]<<width_;
    roi_file.release();

    file_path=calibration_result_dir_+"/calibration.xml";
    cv::FileStorage calibration(file_path, cv::FileStorage::READ);
    std::vector<int> poly_pts;
    std::vector<int> lint_pts;
    cv::FileNode calib_result = calibration["Color_Calib"];
    cv::FileNodeIterator it = calib_result.begin(), it_end = calib_result.end();
    int idx=0;
    for( ; it != it_end; ++it)
    {
        QVector<QPoint> polys_pt;
        (*it)["poly_pts"]>>poly_pts;
        polys_pt.push_back(QPoint(poly_pts[0],poly_pts[1]));
        polys_pt.push_back(QPoint(poly_pts[2],poly_pts[3]));
        polys_pt.push_back(QPoint(poly_pts[4],poly_pts[5]));
        polys_pt.push_back(QPoint(poly_pts[6],poly_pts[7]));
        calib_final_result_[idx]=polys_pt;
        QVector<QPoint> line_pt;
        (*it)["line_pts"]>>lint_pts;
        line_pt.push_back(QPoint(lint_pts[0],0));
        line_pt.push_back(QPoint(lint_pts[1],0));
        calib_Line_final_result_[idx]=line_pt;
        idx++;
    }
    calibration.release();

    load_interim_result();
}

void Dialog::load_interim_result()
{
    circle_diameter_[0]=QPoint(circle_center_.x()-circle_radius_,circle_center_.y());
    circle_diameter_[1]=QPoint(circle_center_.x()+circle_radius_,circle_center_.y());
    small_edge_=QPoint(circle_center_.x()-small_radius_,circle_center_.y());
    for(std::size_t i =0 ;i < calib_final_result_.size();i++)
    {
        QVector<QPoint> pts_tmp;
        QVector<QPoint> & pts = calib_final_result_[i];
        for(std::size_t i=0; i < pts.size() ; i++)
        {
            QPoint pt= QPoint(pts[i].x()*scale_factor_,pts[i].y()*scale_factor_);
            pts_tmp.push_back(pt);
        }
       calib_interim_result_[i]=pts_tmp;

       QVector<QPoint> ps_line_tmp;
       QVector<QPoint> & pts_line = calib_Line_final_result_[i];
       for(std::size_t i=0; i < pts_line.size() ; i++)
       {
           QPoint pt= QPoint(pts_line[i].x()*scale_factor_,256*scale_factor_+15);
           ps_line_tmp.push_back(pt);
       }
       calib_Line_inter_result_[i]=ps_line_tmp;
    }

}

void Dialog::on_Save_ROI_Btn_clicked()
{
    std::string file_path=calibration_result_dir_+"/ROI.xml";
    cv::FileStorage roi_file(file_path, cv::FileStorage::WRITE);
    int start_row = circle_center_.y()-circle_radius_-10;
      if(start_row<0) start_row=0;
    int start_col = circle_center_.x()-circle_radius_-10;
      if(start_col<0) start_col=0;
    QPoint center_tmp =QPoint(circle_center_.x()-start_col, circle_center_.y()-start_row);
    int rows,cols;
    rows=cols=(circle_radius_+10)*2+1;
    if(start_row+rows>orignal_img_[0].rows)
        rows=orignal_img_[0].rows-start_row;
    if(start_col+cols>orignal_img_[0].cols)
        cols=orignal_img_[0].cols-start_col;

    roi_file <<"start_row" << start_row;
    roi_file <<"start_column"<< start_col;
    roi_file <<"center_point_row"<<center_tmp.y();
    roi_file <<"center_point_column"<<center_tmp.x() ;
    roi_file <<"big_radius"<<circle_radius_;
    roi_file <<"small_radius"<< small_radius_;
    roi_file <<"rows"<<rows;
    roi_file <<"cols"<<cols;
    roi_file.release();

}

void Dialog::on_white_wave_Btn_clicked()
{
    static nubot::OmniImage omni_img;
    cv::Mat image;
    cv::cvtColor(orignal_img_[0],image,CV_RGB2BGR);
    if(orignal_img_[0].channels()!=3)
        return ;
    omni_img.setBGRImage(image);
    omni_img.setHSVImage(orignal_img_[1]);
    omni_img.setYUVImage(orignal_img_[2]);
    nubot::Circle circle_tmp;
    circle_tmp.center_=nubot::DPoint2d(circle_center_.x(),circle_center_.y());
    circle_tmp.radius_=circle_radius_;
    omni_img.setBigROI(circle_tmp);
    circle_tmp.radius_=small_radius_;
    omni_img.setSmallROI(circle_tmp);
    omni_img.setHeight(orignal_img_[0].rows);
    omni_img.setWidth(orignal_img_[0].cols);
    static nubot::ScanPoints scan_pts(omni_img);
    static nubot::Whites whites(scan_pts);
    scan_pts.process();
    whites.process();
    if(is_read_whites_)
    {
        whites.img_white_=white_pts_;
        is_read_whites_=false;
    }
    whites.showWhitePoints();
    white_pts_=whites.img_white_;
}


void Dialog::on_Save_image_Btn_clicked()
{
    static int number=0;
    std::ostringstream ostr;
    ostr<<number;
    std::string file_path=calibration_result_dir_+"/"+"pic_save"+ostr.str()+".bmp";
    cv::Mat image;
    cv::cvtColor(orignal_img_[0],image,CV_RGB2BGR);
    cv::imwrite(file_path,image);
    number++;
}


void Dialog::on_load_whites_Btn_clicked()
{
    is_read_whites_=true;
    std::string str=calibration_result_dir_+"/"+"whites.txt";
    white_pts_.clear();
    int tmp1,tmp2;
    std::ifstream whites_read(str.c_str());
    while(whites_read>>tmp1 && whites_read>>tmp2)
       white_pts_.push_back(nubot::DPoint2i(tmp1,tmp2));
    whites_read.close();
}

void Dialog::on_save_whits_Btn_clicked()
{
    std::string str=calibration_result_dir_+"/"+"whites.txt";
    std::ofstream whites_write(str.c_str());
    for(std::size_t i = 0; i < white_pts_.size();i++)
         whites_write<<white_pts_[i].x_<<" "<<white_pts_[i].y_<<" \n";
     whites_write.close();

}

void Dialog::on_Error_table_Btn_clicked()
{
    static nubot::ErrorTable error_table;

    error_table.getErrorTable();
    error_table.getDiffTable();

    std::string file_path=calibration_result_dir_+"/errortable.bin";
    std::ofstream ErrorTable_Write(file_path.c_str(), std::ios::binary|std::ios::out);
    ErrorTable_Write.write((char *)(error_table.DistoMarkLine_),
                            sizeof(double)*error_table.xlong_ * error_table.ylong_);
    ErrorTable_Write.close();

    file_path=calibration_result_dir_+"/Diff_X.bin";
    std::ofstream DiffXTable_Write(file_path.c_str(), std::ios::binary|std::ios::out);
    DiffXTable_Write.write((char *)(error_table.Diff_X_),
                            sizeof(double)*error_table.xlong_ * error_table.ylong_);
    DiffXTable_Write.close();


    file_path=calibration_result_dir_+"/Diff_Y.bin";
    std::ofstream DiffYTable_Write(file_path.c_str(), std::ios::binary|std::ios::out);
    DiffYTable_Write.write((char *)(error_table.Diff_Y_),
                            sizeof(double)*error_table.xlong_ * error_table.ylong_);
    DiffYTable_Write.close();

}
