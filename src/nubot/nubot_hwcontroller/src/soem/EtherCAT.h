#ifndef _EtherCAT_
#define _EtherCAT_

// C与C++混合编程要在CPP文件中加extern “C”关键字，否则链接会出错
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <math.h>
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"
#include "ethercatconfig.h"
#include "ethercatdc.h"

typedef struct PACKED
{
    uint16	   status1;
    uint16	   invalue1;
    uint16	   status2;
    uint16	   invalue2;
    uint16	   status3;
    uint16	   invalue3;
    uint16	   status4;
    uint16	   invalue4;
} in_EL3064t;

in_EL3064t *in_EL3064;
int os;
uint8 ob;
uint16 ob2;

void Elmo_Process_1(int32 *Speed,int32 *Speed_info);
void Elmo_Process_2(int32 *Speed);
void Elmo_Process_error(void);
void Elmo_Process_enable(void);


void EtherCAT_init(const char * ifname);
void Elmo_init(void);
void Base_Enable(int canmove);
void Ballhandle_Enable(int canmove);
void EL2008_set(uint8 num);
void EL2008_clear(uint8 num);

#ifdef __cplusplus
}
#endif

// IO位置定义,跟EtherCat硬件接线有关
#define  Relay_IO               1
#define  IGBT_IO                2
#define  BallLock_Left_IO       3
#define  BallLock_Right_IO      4
#define  ShooterLever_Up_IO     5
#define  ShooterLever_Down_IO   6

#endif
