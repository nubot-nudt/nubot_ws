#ifndef CALIBRATION_DIALOG_H
#define CALIBRATION_DIALOG_H

#include <QDialog>
#include <QFileDialog>
#include "qvector.h"
#include "qpainter.h"
#include <QMouseEvent>
#include "qpen.h"
#include <QLabel>
#include "image_subscribe.h"
#include "opencv2/opencv.hpp"

#include "omniimage.h"
#include "scanpoints.h"
#include "whites.h"
#include "core/core.hpp"

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(image_subscribe & _image, QWidget *parent = 0);
    ~Dialog();

private:
    Ui::Dialog *ui;

public:

    image_subscribe * image_sub_;
    /*! @brief  segment_img[0]--rgb  segment_img[1]--hsi  segment_img[2]--yuv*/
    std::vector <cv::Mat>  orignal_img_;
    std::vector <cv::Mat>  segment_img_;

    int scale_factor_;

private slots:

   void on_receive_image_btn_clicked();
   void on_loadImage_Btn_clicked();
   void on_HSI_RadioBtn_clicked();
   void on_YUV_RadioBtn_clicked();
   void on_segment_result_btn_clicked();
   void on_ball_seg_Btn_clicked();
   void on_obs_seg_Btn_clicked();
   void on_BigCircle_Btn_clicked();
   void on_SmallCircle_Btn_clicked();
   void on_Save_Result_Btn_clicked();
   void on_Load_Result_Btn_clicked();
   void on_Save_ROI_Btn_clicked();
   void on_white_wave_Btn_clicked();
   void on_field_segment_Btn_clicked();
   void on_show_segmentBtn_clicked();
   void on_Save_image_Btn_clicked();
   void on_load_whites_Btn_clicked();
   void on_save_whits_Btn_clicked();

   void on_Error_table_Btn_clicked();

protected:

   void mouseMoveEvent(QMouseEvent * event);
   void mousePressEvent(QMouseEvent * event);
   void mouseReleaseEvent(QMouseEvent * event);
   void paintEvent(QPaintEvent * event);

private:

   /*! @brief  start_pts[0]--rgb    start_pts[1]--hsi  start_pts[2]--yuv*/
   QVector <QPoint>  start_pts_;
   /*! @brief  image[0]--rgb  image[1]--hsi  image[2]--yuv*/
   QVector <QImage>  image;
   /*! @brief  bufferImg[0]--rgb  bufferImg[1]--hsi  bufferImg[2]--yuv*/
   QVector <QImage>  bufferImg_;
   /*! @brief  templeImg[0]--rgb  templeImg[1]--hsi  templeImg[2]--yuv*/
   QVector <QImage>  templeImg_;
   /*! @brief  is_draw_Img[0]--rgb  is_draw_Img[1]--hsi  is_draw_Img[2]--yuv*/
   QVector <bool>    is_draw_Img_;
   /*! @brief  is_screen_update[0]--rgb  is_screen_update[1]--hsi  is_screen_update[2]--yuv*/
   QVector <bool>    is_screen_update_;


   int    Img_selected_;

   QVector <QPoint> poly_pts_;
   QVector <QPoint> Line;
   QVector <QPoint> Line_pts_;

   /*! @brief  calib_interim_result[0]--ball  calib_interim_result[1]--obs  calib_interim_result[2]--white*/
   QVector< QVector < QPoint> > calib_interim_result_;
   /*! @brief  calib_final_result[0]--ball  calib_final_result[1]--obs  calib_final_result[2]--white*/
   QVector< QVector < QPoint> > calib_final_result_;
   /*! @brief  calib_Line_inter_result[0]--ball  calib_Line_inter_result[1]--obs  calib_Line_inter_result[2]--white*/
   QVector< QVector < QPoint> > calib_Line_inter_result_;
   /*! @brief  calib_Line_final_result[0]--ball  calib_Line_final_result[1]--obs  calib_Line_final_result[2]--white*/
   QVector< QVector < QPoint> > calib_Line_final_result_;
   QVector< int > calib_color;

   int  nearest_pt_label_;
   int  nearest_line_pt_;

    /*! the points of circle diameter  */
    bool is_draw_big_circle_;
    bool is_draw_small_circle_;
    QVector <QPoint> circle_diameter_;
    int nearest_diameter_pt_;
    QPoint circle_center_;
    int    circle_radius_;

    QPoint small_edge_;
    int    small_radius_;
    int    height_;
    int    width_;

    std::vector<cv::Mat> color_space_img_;
    unsigned char CTable_[64*64*64];
    std::string calibration_result_dir_;

    std::vector<cv::Mat> color_space_bgr_;
    unsigned char CTable_bgr_[64*64*64];

    cv::Point show_color_;
    QPoint    show_poly_segment_;
    QPoint    show_line_segment_;


    std::vector<nubot::DPoint2i> white_pts_;
    bool is_read_whites_;
 public:
   void paint(QImage & img);
   int  select_nearest_pt(const QVector<QPoint> & pts,QPoint pt,QPoint start_pt);
   bool pnpoly (const QVector<QPoint> & pts, QPoint test_pt);
   bool pnline (const QVector<QPoint> & pts, QPoint test_pt);
   void update_calibration_result();
   void update_calib_interim_result();
   void load_interim_result();
   void update_yuv_table();
   void update_bgr_table();

};

#endif // CALIBRATION_DIALOG_H
