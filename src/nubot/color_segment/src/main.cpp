#include "calibration_dialog.h"
#include <QApplication>
#include "image_subscribe.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc, argv,"calibration_omni");
    ros::NodeHandle node;
    image_subscribe this_image_subscribe(node);
    Dialog w(this_image_subscribe);
    w.show();
    this_image_subscribe.start();
    return a.exec();
}
