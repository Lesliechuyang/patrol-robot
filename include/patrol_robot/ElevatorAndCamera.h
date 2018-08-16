#ifndef _ELEVATOR_AND_CAMERA_H_
#define _ELEVATOR_AND_CAMERA_H_

#include<stdio.h>      /*标准输入输出定义*/ 
#include<string.h>  
#include<stdlib.h>     /*标准函数库定义*/ 
#include<unistd.h>     /*Unix 标准函数定义*/  
#include<sys/types.h>   
#include<sys/stat.h>     
#include<fcntl.h>      /*文件控制定义*/  
#include<termios.h>    /*PPSIX 终端控制定义*/  
#include<errno.h>      /*错误号定义*/  
//#include<delay.h>

#define FALSE  -1  
#define TRUE   0
//*********************************************************//

#define  COMPORT        "/dev/ttyUSB1"   //   "/dev/ttyUSB0"   //"/dev/ttyS0"
#define  BAND_RATE       9600

//*********************************************************//
#define LFIT_TIME     20  //20000 //
#define MAX_LIFT      40
#define COMMAND_LEN   8
#define DEAD_LENTH    2

#define GO_INItial     0
#define GO_UP_START    1
#define GO_UP_STOP     2
#define GO_DOWN_START  3
#define GO_DOWN_STOP   4

void ElevatorInitial(void);

int  ElevatorGoUp(int ms);  //如果ms==0，调用该函数后需要手动调用ElevatorGoUpStop()一次；ms不为0则不用调用ElevatorGoUpStop().
int  ElevatorGoUpStop(void);

int  ElevatorGoDown(int ms);//如果ms==0，调用该函数后需要手动调用ElevatorGoDown()一次；ms不为0则不用调用ElevatorGoDownStop().
int  ElevatorGoDownStop(void);


int  GotoSetPosition(int SetHeight); //会调用参数不为0的上升或下降函数

//*******************************************************************************************************************************************//
#define  INI_ANGLE_X   10.2
#define  INI_ANGLE_Y   1.2
#define  ANGLE_X_STEP  1
#define  ANGLE_Y_STEP  1

typedef enum eCommandType
{
  e_Up=0,
  e_Down,
  e_Left,
  e_Right,
  e_CommandList
}enumCommandTypeDef;

#define  SET_X_ANGLE(Angle)  SetAngle(0 , Angle)
#define  SET_Y_ANGLE(Angle)  SetAngle(1 , Angle)

#define  COMMAND_ALL_STOP   "curl  \"http://admin:admin@192.168.1.99/cgi-bin/hi3510/ptzctrl.cgi?-step=0&-act=stop\""
#define  COMMAND_UP_STEP    "curl  \"http://admin:admin@192.168.1.99/cgi-bin/hi3510/ptzctrl.cgi?-step=1&-act=up\""
#define  COMMAND_DOWN_STEP  "curl  \"http://admin:admin@192.168.1.99/cgi-bin/hi3510/ptzctrl.cgi?-step=1&-act=down\""
#define  COMMAND_LEFT_STEP  "curl  \"http://admin:admin@192.168.1.99/cgi-bin/hi3510/ptzctrl.cgi?-step=1&-act=left\""
#define  COMMAND_RIGHT_STEP "curl  \"http://admin:admin@192.168.1.99/cgi-bin/hi3510/ptzctrl.cgi?-step=1&-act=right\""

#define CameraRightStep();   SendCommand( e_Right  );
#define CameraLeftStep();    SendCommand( e_Left  );
#define CameraUpStep();      SendCommand( e_Up  );
#define CameraDownStep();    SendCommand( e_Down  );

void GoSteps(enumCommandTypeDef eCommand,int times=1);
int SetAngle(char AngleType,float Angle);
//*******************************************************************************************************************************************//

int ElevatorTest(void);
int CameraTest(char *argv);

#endif