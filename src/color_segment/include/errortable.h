#ifndef ERRORTABLE_H
#define ERRORTABLE_H

#include "fieldinformation.h"

namespace nubot
{

class ErrorTable
{
public:
    ErrorTable();

    FieldInformation field_info_;

    int startx_ ;
    int endx_   ;
    int starty_ ;
    int endy_   ;
    int xlong_  ;
    int ylong_  ;

    double * DistoMarkLine_;
    double * Diff_X_;
    double * Diff_Y_;
    void getErrorTable();
    void getDiffTable();

};

}

#endif // ERRORTABLE_H
