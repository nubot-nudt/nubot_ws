#include "EtherCAT.h"

const uint16 can_slave_num=4;
const uint16 EL2008_slave_num=3;


char IOmap[128];
uint32 Cycle_time=0x01312D00;
uint32 Shift_time=0x00B71B00;

char _F800[]={
              0x11,0x00,0x7f,0x01,0x80,0x00,0x20,0x4E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x1e,0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8000[]={
              0x2e,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x01,0x00,0x64,0x00,0x03,0x00,0xD0,0x07,0xd0,0x07,0x05,0x00,0x0a,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8006[]={
              0x01,0x00,0x81,0x05,0x00,0x00,0xff,0x08,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8008[]={
              0x01,0x00,0x01,0x06,0x00,0x00,0xff,0x08,0x00,0x00,0x32,0x00,0x00,0x00
             };//0x32=50,event time!
char _8003[]={
              0x04,0x00,0x00,0x18,0x01,0x04,0x00,0x81,0x05,0x00,0x00,0x00,0x18,0x02,0x01,0x00,0xff,
              0x00,0x14,0x01,0x04,0x00,0x01,0x06,0x00,0x00,0x00,0x14,0x02,0x01,0x00,0xff
             };
char _8010[]={
              0x2e,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x01,0x00,0x64,0x00,0x03,0x00,0xd0,0x07,0xd0,0x07,0x05,0x00,0x0a,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8016[]={
              0x01,0x00,0x82,0x05,0x00,0x00,0xff,0x08,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8018[]={
              0x01,0x00,0x02,0x06,0x00,0x00,0xff,0x08,0x00,0x00,0x32,0x00,0x00,0x00
             };
char _8013[]={
              0x04,0x00,0x00,0x18,0x01,0x04,0x00,0x82,0x05,0x00,0x00,0x00,0x18,0x02,0x01,0x00,0xff,
              0x00,0x14,0x01,0x04,0x00,0x02,0x06,0x00,0x00,0x00,0x14,0x02,0x01,0x00,0xff
             };
char _8020[]={
              0x2e,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x01,0x00,0x64,0x00,0x03,0x00,0xd0,0x07,0xd0,0x07,0x05,0x00,0x0a,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8026[]={
              0x01,0x00,0x83,0x05,0x00,0x00,0xff,0x08,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8028[]={
              0x01,0x00,0x03,0x06,0x00,0x00,0xff,0x08,0x00,0x00,0x32,0x00,0x00,0x00
             };
char _8023[]={
              0x04,0x00,0x00,0x18,0x01,0x04,0x00,0x83,0x05,0x00,0x00,0x00,0x18,0x02,0x01,0x00,0xff,
              0x00,0x14,0x01,0x04,0x00,0x03,0x06,0x00,0x00,0x00,0x14,0x02,0x01,0x00,0xff
             };
char _8030[]={
              0x2e,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x01,0x00,0x64,0x00,0x03,0x00,0xd0,0x07,0xd0,0x07,0x05,0x00,0x0a,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8036[]={
              0x01,0x00,0x84,0x05,0x00,0x00,0xff,0x08,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8038[]={
              0x01,0x00,0x04,0x06,0x00,0x00,0xff,0x08,0x00,0x00,0x32,0x00,0x00,0x00
             };
char _8033[]={
              0x04,0x00,0x00,0x18,0x01,0x04,0x00,0x84,0x05,0x00,0x00,0x00,0x18,0x02,0x01,0x00,0xff,
              0x00,0x14,0x01,0x04,0x00,0x04,0x06,0x00,0x00,0x00,0x14,0x02,0x01,0x00,0xff
             };
char _8040[]={
              0x2e,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x01,0x00,0x64,0x00,0x03,0x00,0xd0,0x07,0xd0,0x07,0x05,0x00,0x0a,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8046[]={
              0x01,0x00,0x85,0x05,0x00,0x00,0xff,0x08,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8048[]={
              0x01,0x00,0x05,0x06,0x00,0x00,0xff,0x08,0x00,0x00,0x32,0x00,0x00,0x00
             };
char _8043[]={
              0x04,0x00,0x00,0x18,0x01,0x04,0x00,0x85,0x05,0x00,0x00,0x00,0x18,0x02,0x01,0x00,0xff,
              0x00,0x14,0x01,0x04,0x00,0x05,0x06,0x00,0x00,0x00,0x14,0x02,0x01,0x00,0xff
             };
char _8050[]={
              0x2e,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x01,0x00,0x64,0x00,0x03,0x00,0xd0,0x07,0xd0,0x07,0x05,0x00,0x0a,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8056[]={
              0x01,0x00,0x86,0x05,0x00,0x00,0xff,0x08,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8058[]={
              0x01,0x00,0x06,0x06,0x00,0x00,0xff,0x08,0x00,0x00,0x32,0x00,0x00,0x00
             };
char _8053[]={
              0x04,0x00,0x00,0x18,0x01,0x04,0x00,0x86,0x05,0x00,0x00,0x00,0x18,0x02,0x01,0x00,0xff,
              0x00,0x14,0x01,0x04,0x00,0x06,0x06,0x00,0x00,0x00,0x14,0x02,0x01,0x00,0xff
             };
char _8060[]={
              0x2e,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x01,0x00,0x64,0x00,0x03,0x00,0xd0,0x07,0xd0,0x07,0x05,0x00,0x0a,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8066[]={
              0x01,0x00,0x87,0x05,0x00,0x00,0xff,0x08,0x00,0x00,0x00,0x00,0x00,0x00
             };
char _8068[]={
              0x01,0x00,0x07,0x06,0x00,0x00,0xff,0x08,0x00,0x00,0x32,0x00,0x00,0x00
             };
char _8063[]={
              0x04,0x00,0x00,0x18,0x01,0x04,0x00,0x87,0x05,0x00,0x00,0x00,0x18,0x02,0x01,0x00,0xff,
              0x00,0x14,0x01,0x04,0x00,0x07,0x06,0x00,0x00,0x00,0x14,0x02,0x01,0x00,0xff
             };
char _1C12[]={0x07,0x00,0x00,0x16,0x01,0x16,0x02,0x16,0x03,0x16,0x04,0x16,0x05,0x16,0x06,0x16};
char _1C13[]={0x0a,0x00,0x00,0x1a,0x01,0x1a,0x02,0x1a,0x03,0x1a,0x04,0x1a,0x05,0x1a,0x06,0x1a,0x82,0x1a,0x83,0x1a,0x84,0x1a};


void Elmo_Process_1(int32 *Speed,int32 *Speed_info)
{
    uint8  *data_in;
    uint64 *data_out;
    int32  indata1,indata2,indata3=0,indata4,indata5=0,indata6=0,indata7=0;
    uint8  indata1_1,indata1_2=0,indata1_3,indata1_4,indata1_5,indata1_6,indata1_7,indata1_8=0;
    uint8  indata2_1,indata2_2=0,indata2_3,indata2_4,indata2_5,indata2_6,indata2_7,indata2_8=0;
    uint8  indata3_1,indata3_2=0,indata3_3,indata3_4,indata3_5,indata3_6,indata3_7,indata3_8=0;
    uint8  indata4_1,indata4_2=0,indata4_3,indata4_4,indata4_5,indata4_6,indata4_7,indata4_8=0;
    uint8  indata5_1,indata5_2=0,indata5_3,indata5_4,indata5_5,indata5_6,indata5_7,indata5_8=0;
    uint8  indata6_1,indata6_2=0,indata6_3,indata6_4,indata6_5,indata6_6,indata6_7,indata6_8=0;
    uint8  indata7_1,indata7_2=0,indata7_3,indata7_4,indata7_5,indata7_6,indata7_7,indata7_8=0;

    if(Speed_info!=0)
    {
        /*Read Speed*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        *data_out++ = 0x0000000000606940;
        *data_out++ = 0x0000000000606940;
        *data_out++ = 0x0000000000606940;
        *data_out++ = 0x0000000000606940;
        //*data_out++ = 0x0000000000606940;
        //*data_out++ = 0x0000000000606940;
        //*data_out++ = 0x0000000000606940;
        ec_send_processdata();

        usleep(17000);
    }

    /*Set Speed*/
    data_out = (uint64 *)ec_slave[can_slave_num].outputs;
    *data_out++ = 0x000000000060FF22+((int64)Speed[0]<<32)*33;//33是Composer给出的
    *data_out++ = 0x000000000060FF22+((int64)Speed[1]<<32)*33;
    *data_out++ = 0x000000000060FF22+((int64)Speed[2]<<32)*33;
    *data_out++ = 0x000000000060FF22+((int64)Speed[3]<<32)*33;
    //*data_out++ = 0x000000000060FF22+((int64)Speed[4]<<32)*33;
    //*data_out++ = 0x000000000060FF22+((int64)Speed[5]<<32)*33;
    //*data_out++ = 0x000000000060FF22+((int64)Speed[6]<<32)*33;
    ec_send_processdata();

    if(Speed_info!=0)
    {
        ec_receive_processdata(500);
        data_in  = ec_slave[can_slave_num].inputs;

        indata1_1=*data_in++;indata1_2=*data_in++;indata1_3=*data_in++;indata1_4=*data_in++;
        indata1_5=*data_in++;indata1_6=*data_in++;indata1_7=*data_in++;indata1_8=*data_in++;
        if(indata1_1==0x42&&indata1_2==0x69&&indata1_3==0x60&&indata1_4==0x00)
        {
            if(indata1_8==0xFF)
            {
                indata1=(0xFF-indata1_5)+((0xFF-indata1_6)<<8)+((0xFF-indata1_7)<<16)+((0xFF-indata1_8)<<32);
                indata1=indata1*-1;
            }
            else
            {
                indata1=indata1_5+(indata1_6<<8)+(indata1_7<<16)+(indata1_8<<32);
            }
            indata1=indata1/33;
        }
        else indata1=0;

        indata2_1=*data_in++;indata2_2=*data_in++;indata2_3=*data_in++;indata2_4=*data_in++;
        indata2_5=*data_in++;indata2_6=*data_in++;indata2_7=*data_in++;indata2_8=*data_in++;
        if(indata2_1==0x42&&indata2_2==0x69&&indata2_3==0x60&&indata2_4==0x00)
        {
            if(indata2_8==0xFF)
            {
                indata2=(0xFF-indata2_5)+((0xFF-indata2_6)<<8)+((0xFF-indata2_7)<<16)+((0xFF-indata2_8)<<32);
                indata2=indata2*-1;
            }
            else
            {
                indata2=indata2_5+(indata2_6<<8)+(indata2_7<<16)+(indata2_8<<32);
            }
            indata2=indata2/33;
        }
        else indata2=0;

        indata3_1=*data_in++;indata3_2=*data_in++;indata3_3=*data_in++;indata3_4=*data_in++;
        indata3_5=*data_in++;indata3_6=*data_in++;indata3_7=*data_in++;indata3_8=*data_in++;
        if(indata3_1==0x42&&indata3_2==0x69&&indata3_3==0x60&&indata3_4==0x00)
        {
            if(indata3_8==0xFF)
            {
                indata3=(0xFF-indata3_5)+((0xFF-indata3_6)<<8)+((0xFF-indata3_7)<<16)+((0xFF-indata3_8)<<32);
                indata3=indata3*-1;
            }
            else
            {
                indata3=indata3_5+(indata3_6<<8)+(indata3_7<<16)+(indata3_8<<32);
            }
            indata3=indata3/33;
        }
        else indata3=0;

        indata4_1=*data_in++;indata4_2=*data_in++;indata4_3=*data_in++;indata4_4=*data_in++;
        indata4_5=*data_in++;indata4_6=*data_in++;indata4_7=*data_in++;indata4_8=*data_in++;
        if(indata4_1==0x42&&indata4_2==0x69&&indata4_3==0x60&&indata4_4==0x00)
        {
            if(indata4_8==0xFF)
            {
                indata4=(0xFF-indata4_5)+((0xFF-indata4_6)<<8)+((0xFF-indata4_7)<<16)+((0xFF-indata4_8)<<32);
                indata4=indata4*-1;
            }
            else
            {
                indata4=indata4_5+(indata4_6<<8)+(indata4_7<<16)+(indata4_8<<32);
            }
            indata4=indata4/33;
        }
        else indata4=0;

        indata5_1=*data_in++;indata5_2=*data_in++;indata5_3=*data_in++;indata5_4=*data_in++;
        indata5_5=*data_in++;indata5_6=*data_in++;indata5_7=*data_in++;indata5_8=*data_in++;
        if(indata5_1==0x42&&indata5_2==0x69&&indata5_3==0x60&&indata5_4==0x00)
        {
            if(indata5_8==0xFF)
            {
                indata5=(0xFF-indata5_5)+((0xFF-indata5_6)<<8)+((0xFF-indata5_7)<<16)+((0xFF-indata5_8)<<32);
                indata5=indata5*-1;
            }
            else
            {
                indata5=indata5_5+(indata5_6<<8)+(indata5_7<<16)+(indata5_8<<32);
            }
            indata5=indata5/33;
        }
        else indata5=0;

        indata6_1=*data_in++;indata6_2=*data_in++;indata6_3=*data_in++;indata6_4=*data_in++;
        indata6_5=*data_in++;indata6_6=*data_in++;indata6_7=*data_in++;indata6_8=*data_in++;
        if(indata6_1==0x42&&indata6_2==0x69&&indata6_3==0x60&&indata6_4==0x00)
        {
            if(indata6_8==0xFF)
            {
                indata6=(0xFF-indata6_5)+((0xFF-indata6_6)<<8)+((0xFF-indata6_7)<<16)+((0xFF-indata6_8)<<32);
                indata6=indata6*-1;
            }
            else
            {
                indata6=indata6_5+(indata6_6<<8)+(indata6_7<<16)+(indata6_8<<32);
            }
            indata6=indata6/33;
        }
        else indata6=0;

        indata7_1=*data_in++;indata7_2=*data_in++;indata7_3=*data_in++;indata7_4=*data_in++;
        indata7_5=*data_in++;indata7_6=*data_in++;indata7_7=*data_in++;indata7_8=*data_in++;
        if(indata7_1==0x42&&indata7_2==0x69&&indata7_3==0x60&&indata7_4==0x00)
        {
            if(indata7_8==0xFF)
            {
                indata7=(0xFF-indata7_5)+((0xFF-indata7_6)<<8)+((0xFF-indata7_7)<<16)+((0xFF-indata7_8)<<32);
                indata7=indata7*-1;
            }
            else
            {
                indata7=indata7_5+(indata7_6<<8)+(indata7_7<<16)+(indata7_8<<32);
            }
            indata7=indata7/33;
        }
        else indata7=0;

        *Speed_info++=indata1;*Speed_info++=indata2;*Speed_info++=indata3;*Speed_info++=indata4;
        *Speed_info++=indata5;*Speed_info++=indata6;*Speed_info++=indata7;
    }
}

void Elmo_Process_2(int32 *Speed)
{
    uint64 *data_out;
    /*Set Speed*/
    data_out = (uint64 *)ec_slave[can_slave_num].outputs;
    data_out++ ;
    data_out++ ;
    data_out++ ;
    data_out++ ;
    *data_out++ = 0x000000000060FF22+((int64)Speed[0]<<32)*33;
    *data_out++ = 0x000000000060FF22+((int64)Speed[1]<<32)*33;
    ec_send_processdata();
    ec_receive_processdata(500);
}


void Elmo_Process_error(void)
{
    uint64 *data_out;
    /*EREASE ERROR*/
    data_out = (uint64 *)ec_slave[can_slave_num].outputs;
    *data_out++ = 0x0000008000604022;
    *data_out++ = 0x0000008000604022;
    *data_out++ = 0x0000008000604022;
    *data_out++ = 0x0000008000604022;
    ec_send_processdata();
    usleep(2000);

}


void Elmo_Process_enable(void)
{
    uint64 *data_out;
    /*ENABLE*/
    data_out = (uint64 *)ec_slave[can_slave_num].outputs;
    *data_out++ = 0x0000000F00604022;
    *data_out++ = 0x0000000F00604022;
    *data_out++ = 0x0000000F00604022;
    *data_out++ = 0x0000000F00604022;
    ec_send_processdata();
    usleep(2000);

}

void EtherCAT_init(const char * ifname)
{
    if (ec_init(ifname))
    {
        if (ec_config_init(FALSE)>0)
        {
            printf("%d slaves found and configured.\n",ec_slavecount);

            ec_SDOwrite(can_slave_num,0x1C32,0x02,FALSE,sizeof(Cycle_time),&Cycle_time,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x1C33,0x03,FALSE,sizeof(Shift_time),&Shift_time,EC_TIMEOUTRXM);

            ec_SDOwrite(can_slave_num,0x8000,0x00,TRUE,sizeof(_8000),&_8000,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8006,0x00,TRUE,sizeof(_8006),&_8006,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8008,0x00,TRUE,sizeof(_8008),&_8008,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8003,0x00,TRUE,sizeof(_8003),&_8003,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8010,0x00,TRUE,sizeof(_8010),&_8010,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8016,0x00,TRUE,sizeof(_8016),&_8016,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8018,0x00,TRUE,sizeof(_8018),&_8018,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8013,0x00,TRUE,sizeof(_8013),&_8013,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8020,0x00,TRUE,sizeof(_8020),&_8020,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8026,0x00,TRUE,sizeof(_8026),&_8026,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8028,0x00,TRUE,sizeof(_8028),&_8028,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8023,0x00,TRUE,sizeof(_8023),&_8023,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8030,0x00,TRUE,sizeof(_8030),&_8030,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8036,0x00,TRUE,sizeof(_8036),&_8036,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8038,0x00,TRUE,sizeof(_8038),&_8038,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8033,0x00,TRUE,sizeof(_8033),&_8033,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8040,0x00,TRUE,sizeof(_8040),&_8040,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8046,0x00,TRUE,sizeof(_8046),&_8046,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8048,0x00,TRUE,sizeof(_8048),&_8048,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8043,0x00,TRUE,sizeof(_8043),&_8043,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8050,0x00,TRUE,sizeof(_8050),&_8050,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8056,0x00,TRUE,sizeof(_8056),&_8056,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8058,0x00,TRUE,sizeof(_8058),&_8058,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8053,0x00,TRUE,sizeof(_8053),&_8053,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8060,0x00,TRUE,sizeof(_8060),&_8060,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8066,0x00,TRUE,sizeof(_8066),&_8066,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8068,0x00,TRUE,sizeof(_8068),&_8068,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8063,0x00,TRUE,sizeof(_8063),&_8063,EC_TIMEOUTRXM);

            ec_SDOwrite(can_slave_num,0x1C12,0x00,TRUE,sizeof(_1C12),&_1C12,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x1C13,0x00,TRUE,sizeof(_1C13),&_1C13,EC_TIMEOUTRXM);

            ec_config_map(&IOmap);
            ec_configdc();
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

            ec_slave[0].state=EC_STATE_OPERATIONAL;
            ec_writestate(0);
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

            ec_init(ifname);
            ec_config_init(FALSE);

            ec_SDOwrite(can_slave_num,0x1C32,0x02,FALSE,sizeof(Cycle_time),&Cycle_time,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x1C33,0x03,FALSE,sizeof(Shift_time),&Shift_time,EC_TIMEOUTRXM);

            ec_SDOwrite(can_slave_num,0xF800,0x00,TRUE,sizeof(_F800),&_F800,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8000,0x00,TRUE,sizeof(_8000),&_8000,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8006,0x00,TRUE,sizeof(_8006),&_8006,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8008,0x00,TRUE,sizeof(_8008),&_8008,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8003,0x00,TRUE,sizeof(_8003),&_8003,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8010,0x00,TRUE,sizeof(_8010),&_8010,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8016,0x00,TRUE,sizeof(_8016),&_8016,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8018,0x00,TRUE,sizeof(_8018),&_8018,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8013,0x00,TRUE,sizeof(_8013),&_8013,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8020,0x00,TRUE,sizeof(_8020),&_8020,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8026,0x00,TRUE,sizeof(_8026),&_8026,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8028,0x00,TRUE,sizeof(_8028),&_8028,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8023,0x00,TRUE,sizeof(_8023),&_8023,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8030,0x00,TRUE,sizeof(_8030),&_8030,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8036,0x00,TRUE,sizeof(_8036),&_8036,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8038,0x00,TRUE,sizeof(_8038),&_8038,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8033,0x00,TRUE,sizeof(_8033),&_8033,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8040,0x00,TRUE,sizeof(_8040),&_8040,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8046,0x00,TRUE,sizeof(_8046),&_8046,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8048,0x00,TRUE,sizeof(_8048),&_8048,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8043,0x00,TRUE,sizeof(_8043),&_8043,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8050,0x00,TRUE,sizeof(_8050),&_8050,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8056,0x00,TRUE,sizeof(_8056),&_8056,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8058,0x00,TRUE,sizeof(_8058),&_8058,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8053,0x00,TRUE,sizeof(_8053),&_8053,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8060,0x00,TRUE,sizeof(_8060),&_8060,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8066,0x00,TRUE,sizeof(_8066),&_8066,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8068,0x00,TRUE,sizeof(_8068),&_8068,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x8063,0x00,TRUE,sizeof(_8063),&_8063,EC_TIMEOUTRXM);

            ec_SDOwrite(can_slave_num,0x1C12,0x00,TRUE,sizeof(_1C12),&_1C12,EC_TIMEOUTRXM);
            ec_SDOwrite(can_slave_num,0x1C13,0x00,TRUE,sizeof(_1C13),&_1C13,EC_TIMEOUTRXM);

            ec_config_map(&IOmap);
            ec_configdc();
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

            in_EL3064 = (in_EL3064t*) ec_slave[2].inputs;
            /* set filter on for EL3102 Index 0x4061:03 = 1 */
            os=sizeof(ob); ob=1;
            ec_SDOwrite(2,0x8010,06,FALSE,os,&ob,EC_TIMEOUTRXM);
            ec_SDOwrite(2,0x8020,06,FALSE,os,&ob,EC_TIMEOUTRXM);
            ec_SDOwrite(2,0x8030,06,FALSE,os,&ob,EC_TIMEOUTRXM);
            ec_SDOwrite(2,0x8040,06,FALSE,os,&ob,EC_TIMEOUTRXM);
            /* set filter to IIR level 8 */
            os=sizeof(ob2); ob2=9;
            ec_SDOwrite(2,0x8010,15,FALSE,os,&ob2,EC_TIMEOUTRXM);
            ec_SDOwrite(2,0x8020,15,FALSE,os,&ob2,EC_TIMEOUTRXM);
            ec_SDOwrite(2,0x8030,15,FALSE,os,&ob2,EC_TIMEOUTRXM);
            ec_SDOwrite(2,0x8040,15,FALSE,os,&ob2,EC_TIMEOUTRXM);

            ec_slave[0].state=EC_STATE_OPERATIONAL;
            ec_writestate(0);
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

            Elmo_init();
        }
        else
        {
            printf("Failed to open EtherCAT, please check the wire connection !\n");
        }
    }
}

void Elmo_init(void)
{
    uint64 *data_out;
    /*EREASE ERROR*/
    data_out = (uint64 *)ec_slave[can_slave_num].outputs;
    *data_out++ = 0x0000008000604022;
    *data_out++ = 0x0000008000604022;
    *data_out++ = 0x0000008000604022;
    *data_out++ = 0x0000008000604022;
    *data_out++ = 0x0000008000604022;
    *data_out++ = 0x0000008000604022;
    *data_out++ = 0x0000008000604022;
    ec_send_processdata();
    usleep(50000);

    /*SHUT DOWN*/
    data_out = (uint64 *)ec_slave[can_slave_num].outputs;
    *data_out++ = 0x0000000600604022;
    *data_out++ = 0x0000000600604022;
    *data_out++ = 0x0000000600604022;
    *data_out++ = 0x0000000600604022;
    *data_out++ = 0x0000000600604022;
    *data_out++ = 0x0000000600604022;
    *data_out++ = 0x0000000600604022;
    ec_send_processdata();
    usleep(20000);

    /*PVM MODE*/
    data_out = (uint64 *)ec_slave[can_slave_num].outputs;
    *data_out++ = 0x0000000300606022;
    *data_out++ = 0x0000000300606022;
    *data_out++ = 0x0000000300606022;
    *data_out++ = 0x0000000300606022;
    *data_out++ = 0x0000000300606022;
    *data_out++ = 0x0000000300606022;
    *data_out++ = 0x0000000300606022;
    ec_send_processdata();
    usleep(20000);

    /*SWITCH ON*/
    data_out = (uint64 *)ec_slave[can_slave_num].outputs;
    *data_out++ = 0x0000000700604022;
    *data_out++ = 0x0000000700604022;
    *data_out++ = 0x0000000700604022;
    *data_out++ = 0x0000000700604022;
    *data_out++ = 0x0000000700604022;
    *data_out++ = 0x0000000700604022;
    *data_out++ = 0x0000000700604022;
    ec_send_processdata();
    usleep(20000);

    /*ENABLE*/
    data_out = (uint64 *)ec_slave[can_slave_num].outputs;
    *data_out++ = 0x0000000F00604022;
    *data_out++ = 0x0000000F00604022;
    *data_out++ = 0x0000000F00604022;
    *data_out++ = 0x0000000F00604022;
    *data_out++ = 0x0000000F00604022;
    *data_out++ = 0x0000000F00604022;
    *data_out++ = 0x0000000F00604022;
    ec_send_processdata();
    usleep(20000);
}

void Base_Enable(int canmove)
{
    uint64 *data_out;
    if(canmove)
    {
        /*EREASE ERROR*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        *data_out++ = 0x0000008000604022;
        *data_out++ = 0x0000008000604022;
        *data_out++ = 0x0000008000604022;
        *data_out++ = 0x0000008000604022;
        ec_send_processdata();
        usleep(50000);

        /*SHUT DOWN*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        *data_out++ = 0x0000000600604022;
        *data_out++ = 0x0000000600604022;
        *data_out++ = 0x0000000600604022;
        *data_out++ = 0x0000000600604022;
        ec_send_processdata();
        usleep(20000);

        /*PVM MODE*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        *data_out++ = 0x0000000300606022;
        *data_out++ = 0x0000000300606022;
        *data_out++ = 0x0000000300606022;
        *data_out++ = 0x0000000300606022;
        ec_send_processdata();
        usleep(20000);

        /*SWITCH ON*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        *data_out++ = 0x0000000700604022;
        *data_out++ = 0x0000000700604022;
        *data_out++ = 0x0000000700604022;
        *data_out++ = 0x0000000700604022;
        ec_send_processdata();
        usleep(20000);

        /*ENABLE*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        *data_out++ = 0x0000000F00604022;
        *data_out++ = 0x0000000F00604022;
        *data_out++ = 0x0000000F00604022;
        *data_out++ = 0x0000000F00604022;
        ec_send_processdata();
        usleep(20000);
    }
    else
    {
        /*EREASE ERROR*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        *data_out++ = 0x0000008000604022;
        *data_out++ = 0x0000008000604022;
        *data_out++ = 0x0000008000604022;
        *data_out++ = 0x0000008000604022;
        ec_send_processdata();
        usleep(50000);

        /*SHUT DOWN*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        *data_out++ = 0x0000000600604022;
        *data_out++ = 0x0000000600604022;
        *data_out++ = 0x0000000600604022;
        *data_out++ = 0x0000000600604022;
        ec_send_processdata();
        usleep(20000);

        /*PVM MODE*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        *data_out++ = 0x0000000300606022;
        *data_out++ = 0x0000000300606022;
        *data_out++ = 0x0000000300606022;
        *data_out++ = 0x0000000300606022;
        ec_send_processdata();
        usleep(20000);

        /*SWITCH ON*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        *data_out++ = 0x0000000700604022;
        *data_out++ = 0x0000000700604022;
        *data_out++ = 0x0000000700604022;
        *data_out++ = 0x0000000700604022;
        ec_send_processdata();
        usleep(20000);
    }
}


void Ballhandle_Enable(int canmove)
{
    uint64 *data_out;
    if(canmove)
    {
        /*EREASE ERROR*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        data_out++;
        data_out++;
        data_out++;
        data_out++;
        *data_out++ = 0x0000008000604022;
        *data_out++ = 0x0000008000604022;
        ec_send_processdata();
        usleep(50000);

        /*SHUT DOWN*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        data_out++;
        data_out++;
        data_out++;
        data_out++;
        *data_out++ = 0x0000000600604022;
        *data_out++ = 0x0000000600604022;
        ec_send_processdata();
        usleep(20000);

        /*PVM MODE*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        data_out++;
        data_out++;
        data_out++;
        data_out++;
        *data_out++ = 0x0000000300606022;
        *data_out++ = 0x0000000300606022;
        ec_send_processdata();
        usleep(20000);

        /*SWITCH ON*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        data_out++;
        data_out++;
        data_out++;
        data_out++;
        *data_out++ = 0x0000000700604022;
        *data_out++ = 0x0000000700604022;
        ec_send_processdata();
        usleep(20000);

        /*ENABLE*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        data_out++;
        data_out++;
        data_out++;
        data_out++;
        *data_out++ = 0x0000000F00604022;
        *data_out++ = 0x0000000F00604022;
        ec_send_processdata();
        usleep(20000);
    }
    else
    {
        /*EREASE ERROR*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        data_out++;
        data_out++;
        data_out++;
        data_out++;
        *data_out++ = 0x0000008000604022;
        *data_out++ = 0x0000008000604022;
        ec_send_processdata();
        usleep(50000);

        /*SHUT DOWN*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        data_out++;
        data_out++;
        data_out++;
        data_out++;
        *data_out++ = 0x0000000600604022;
        *data_out++ = 0x0000000600604022;
        ec_send_processdata();
        usleep(20000);

        /*PVM MODE*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        data_out++;
        data_out++;
        data_out++;
        data_out++;
        *data_out++ = 0x0000000300606022;
        *data_out++ = 0x0000000300606022;
        ec_send_processdata();
        usleep(20000);

        /*SWITCH ON*/
        data_out = (uint64 *)ec_slave[can_slave_num].outputs;
        data_out++;
        data_out++;
        data_out++;
        data_out++;
        *data_out++ = 0x0000000700604022;
        *data_out++ = 0x0000000700604022;
        ec_send_processdata();
        usleep(20000);
    }
}



void EL2008_set(uint8 num)
{
    uint8 *data_out;
    uint8 indata=0;
    data_out = ec_slave[EL2008_slave_num].outputs;
    indata=*data_out++;
    data_out = ec_slave[EL2008_slave_num].outputs;

   *data_out =indata|(1<<(num-1));


    ec_send_processdata();
}

void EL2008_clear(uint8 num)
{
    uint8 *data_out;
    uint8 indata=0;
    data_out = ec_slave[EL2008_slave_num].outputs;
    indata=*data_out++;
    data_out = ec_slave[EL2008_slave_num].outputs;

    *data_out =(255-(1<<(num-1)))&indata;

    ec_send_processdata();
}

