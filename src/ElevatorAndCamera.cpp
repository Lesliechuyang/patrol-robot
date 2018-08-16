#include <patrol_robot/ElevatorAndCamera.h>

//#include"ElevatorAndCamera.h"
//***************************************************************************//

//***************************************************************************//


static unsigned char UART_rcv_buf[1024]; 
int fd; 

void delay_sec(unsigned int n)
{
	sleep(n);
}

void delay_4ms(unsigned int n)
{			//delay()函数，原型为extern void delay(unsigned int msec);它可以延时msec*4毫秒
   // msleep
}

void delay_ms(int nms) 
{
  for(int i=0;i<nms;i++)
   usleep(1000);   
}
int UART_Open(int fd,const  char* port)  
{  
     fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);  
     if (FALSE == fd)  
     {  
          perror("Can't Open Serial Port");  
          return(FALSE);  
     }  
    //恢复串口为阻塞状态                          
    if(fcntl(fd, F_SETFL, 0) < 0){ 
	printf("fcntl failed!\n");  
	return(FALSE);
    }       
    else{  
	printf("fcntl Success!\n");
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));  
    }  
  //测试是否为终端设备      
  if(0 == isatty(STDIN_FILENO)) {  
	printf("standard input is not a terminal device\n");  
	 return(FALSE);  
  }  
  else {  
    printf("isatty success!\n");  
  }                
  printf("fd->open=%d\n",fd);  
  return fd;  
}  
void UART_Close(int fd)  
{  
    close(fd);  
}  
int UART_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)  
{  
         int   i;  
         int   status;  
         int   speed_arr[] = { B115200, B38400,B19200, B9600, B4800, B2400, B1200, B300};  
         int   name_arr[] = {115200,  38400, 19200,  9600,  4800,  2400,  1200,  300};  
               
        struct termios options;  
         
        if  ( tcgetattr( fd,&options)  !=  0)  
           {  
              perror("SetupSerial 1");      
              return(FALSE);   
           }  
        
        //设置串口输入波特率和输出波特率  
        for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)  
        {  
              if  (speed == name_arr[i])  
              {               
                    cfsetispeed(&options, speed_arr[i]);   
                    cfsetospeed(&options, speed_arr[i]);    
               }  
        }       
         
        //修改控制模式，保证程序不会占用串口  
        options.c_cflag |= CLOCAL;  
        //修改控制模式，使得能够从串口中读取输入数据  
        options.c_cflag |= CREAD;  
        
        //设置数据流控制  
        switch(flow_ctrl)  
        {  
            
           case 0 ://不使用流控制  
                  options.c_cflag &= ~CRTSCTS;  
                  break;     
            
           case 1 ://使用硬件流控制  
                  options.c_cflag |= CRTSCTS;  
                  break;  
           case 2 ://使用软件流控制  
                  options.c_cflag |= IXON | IXOFF | IXANY;  
                  break;  
	  default:  //不使用流控制 
		  options.c_cflag &= ~CRTSCTS;  
                  break;   
        }  
        //设置数据位  
        //屏蔽其他标志位  
        options.c_cflag &= ~CSIZE;  
        switch (databits)  
        {    
           case 5    :  
                         options.c_cflag |= CS5;  
                         break;  
           case 6    :  
                         options.c_cflag |= CS6;  
                         break;  
           case 7    :      
                     options.c_cflag |= CS7;  
                     break;  
           case 8:      
                     options.c_cflag |= CS8;  
                     break;    
           default:     
                     fprintf(stderr,"Unsupported data size\n");  
                     return (FALSE);   
        }  
        //设置校验位  
        switch (parity)  
        {    
           case 'n':  
           case 'N': //无奇偶校验位。  
                     options.c_cflag &= ~PARENB;   
                     options.c_iflag &= ~INPCK;      
                     break;   
           case 'o':    
           case 'O'://设置为奇校验      
                     options.c_cflag |= (PARODD | PARENB);   
                     options.c_iflag |= INPCK;               
                     break;   
           case 'e':   
           case 'E'://设置为偶校验    
                     options.c_cflag |= PARENB;         
                     options.c_cflag &= ~PARODD;         
                     options.c_iflag |= INPCK;        
                     break;  
           case 's':  
           case 'S': //设置为空格   
                     options.c_cflag &= ~PARENB;  
                     options.c_cflag &= ~CSTOPB;  
                     break;   
            default:    
                     fprintf(stderr,"Unsupported parity\n");      
                     return (FALSE);   
        }   
        // 设置停止位   
        switch (stopbits)  
        {    
           case 1:     
                     options.c_cflag &= ~CSTOPB; break;   
           case 2:     
                     options.c_cflag |= CSTOPB; break;  
           default:     
                           fprintf(stderr,"Unsupported stop bits\n");   
                           return (FALSE);  
        }  
         
      //修改输出模式，原始数据输出  
      options.c_oflag &= ~OPOST;  
        
      options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的  
    //options.c_lflag &= ~(ISIG | ICANON);  
         
        //设置等待时间和最小接收字符  
        options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */    
        options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */  
         
        //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读  
        tcflush(fd,TCIFLUSH);  
         
        //激活配置 (将修改后的termios数据设置到串口中）  
        if (tcsetattr(fd,TCSANOW,&options) != 0)    
        {  
            perror("com set error!\n");    
            return (FALSE);   
        }  
        return (TRUE);   
}  
void  FlushCOMBuf(char WhichBuf=0)
{
	switch (WhichBuf)
	{
		case (0):tcflush(fd,TCIFLUSH); break;
		case (1):tcflush(fd,TCOFLUSH); break;
		case (2):tcflush(fd,TCIOFLUSH); break;
		default:break;
	}
}
int UART_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)  
{  
        int err;  
        if (UART_Set(fd,speed,0,8,1,'N') == FALSE)  
        {                                                           
            return FALSE;  
         }  
        else  
        {  
            return  TRUE;  
        }  
}  
int UART_Recv( unsigned char *UART_rcv_buf,int data_len)  
{  
        int len,fs_sel;  
        fd_set fs_read;  
         
        struct timeval time;  
         
        FD_ZERO(&fs_read);  
        FD_SET(fd,&fs_read);  
         
        time.tv_sec = 10;  
        time.tv_usec = 0;  
         
        //使用select实现串口的多路通信  
        fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);  
        if(fs_sel)  
        {  
              len = read(fd,UART_rcv_buf,data_len);  
              //printf("Recv,len = %d fs_sel = %d\r\n",len,fs_sel);  
              return len;  
        }  
        else  
        {  
              printf("UART_Recv wrong!\r\n");  
              return FALSE;  
         }       
}  


int UART_Send(unsigned char *send_buf,int data_len)  
{  
        int len = 0;  
         
        len = write(fd,send_buf,data_len);  
        if (len == data_len )  
         {  
             return len;  
         }       
        else     
        {  
                     
            tcflush(fd,TCOFLUSH);  
            return FALSE;  
        }  
         
}  
void UART_Recv_And_Disp_Data(void)
{
         int len; 
		 len = UART_Recv( UART_rcv_buf,50);  
         if(len > 0)  
         {  
           UART_rcv_buf[len] = '\0';
	       printf("\r\nreceive %d data:\r\n",len);   

		   for(short i=0;i<len;i++)
		   {
			   printf("%#02X ",UART_rcv_buf[i]);
			}
			printf("\r\n\r\n");
         }  
         else  
         {  
             printf("UART_Recv_And_Disp_Data,cannot receive data\r\n\r\n");  
         } 
}
//-----------------------------------------------------------------------------
int com_initial(void)
{
  int err,err2,uart_res,uart_len;               //返回调用函数的状态 
  fd = UART_Open(fd,COMPORT); //打开串口，返回文件描述符 
  err=fd;
  if(FALSE==err)
  {
  	printf("打开串口失败0\r\n"); 
  	return TRUE;
  }
 // else	printf("打开串口成功0\r\n");

 err2 = UART_Init(fd,BAND_RATE,0,8,1,'N');
 while(FALSE == err2 || FALSE == fd)
 {
	printf("初始化串口失败0!\r\n\r\n"); 
       err2 = UART_Init(fd,BAND_RATE,0,8,1,'N');  
       
      delay_ms(3000);
 } 

  if(FALSE==err|| FALSE == fd || FALSE==err2)
  {
	printf("初始化串口失败0\r\n");  
	return FALSE;
  }
 // else      printf("初始化串口成功0\r\n"); 

  return TRUE;
}
//***********************************************************************************//

volatile int current_operation=GO_INItial;
volatile int current_position=0;

unsigned char CommandGoDownStart[8]={0xFE,0X05,0X00,0X00, 0X00,0X00,0XD9,0XC5};//FE 05 00 00 00 00 D9 C5
unsigned char CommandGoDownStop[8] ={0xFE,0X05,0X00,0X00, 0XFF,0X00,0X98,0X35};//FE 05 00 00 FF 00 98 35

unsigned char CommandGoUpStart[8]  ={0xFE,0x05,0x00,0x01, 0x00,0x00,0x88,0x05};//FE 05 00 01 00 00 88 05
unsigned char CommandGoUpStop[8]   ={0xFE,0x05,0x00,0x01, 0xFF,0x00,0xC9,0xF5};//FE 05 00 01 FF 00 C9 F5


int ElevatorGoUp(int ms=0)
{
  int Res;
  if(0==ms){
    Res=UART_Send(CommandGoUpStart,COMMAND_LEN);
    if(Res!=COMMAND_LEN){
      printf("GoUp Send Error\r\n");
      return 1;
    } 

    current_operation= GO_UP_START;
  }
  else if(0<ms){
    Res=UART_Send(CommandGoUpStart,COMMAND_LEN);
    if(Res!=COMMAND_LEN){
      return 3;
    }
    current_operation= GO_UP_START;
    //delay_ms(ms);
	  delay_sec(ms);  //delay_4ms(ms/4);
    Res=UART_Send(CommandGoUpStop,COMMAND_LEN);
    if(Res!=COMMAND_LEN){
      return 4;
    }
    current_operation=GO_UP_STOP;
  }
  return 0;
}

int ElevatorGoUpStop(void)
{
    int Res;
    Res=UART_Send(CommandGoUpStop,COMMAND_LEN);
    if(Res!=COMMAND_LEN){
      printf("GoUpStop, Send CommandGoUpStop Error\r\n");
      return 1;
    }
 
    return 0;
}


int ElevatorGoDown(int ms=0)
{
  int Res;
  if(0==ms){
    Res=UART_Send(CommandGoDownStart,COMMAND_LEN);
    if(Res!=COMMAND_LEN){
      printf("GoDown Send Error\r\n");
      return 1;
    } 

    current_operation= GO_DOWN_START;
  }
  else if(0<ms){
    Res=UART_Send(CommandGoDownStart,COMMAND_LEN);
    if(Res!=COMMAND_LEN){
      return 3;
    }
    current_operation= GO_DOWN_START;
    //delay_ms(ms);
	  //delay_4ms(ms/4);
    delay_sec(ms);
    Res=UART_Send(CommandGoDownStop,COMMAND_LEN);
    if(Res!=COMMAND_LEN){
      printf("ElevatorGoDown,UART_Send(CommandGoDownStop,COMMAND_LEN); ERROR\r\n");
      return 4;
    }
    
    current_operation=GO_DOWN_STOP;
  }
  return 0;
}
int ElevatorGoDownStop(void)
{
    int Res;
    Res=UART_Send(CommandGoDownStop,COMMAND_LEN);
    if(Res!=COMMAND_LEN){
      printf("GoDownStop, Send CommandGoDownStop Error\r\n");
      return 1;
    }
    return 0;
}
int ElevatorStop(void)
{
  if(current_operation==GO_INItial){
    printf("current_operation==GO_INItial\r\n\r\n");
    return 1;
  }
  int Res;
  switch(current_operation)
  {
    case (GO_DOWN_START):
        Res=ElevatorGoDownStop();
        if(Res!=0){printf("stop,GO_DOWN_START ERROR\r\n"); return 2;}
        current_operation=GO_DOWN_STOP;
        break;
    case (GO_DOWN_STOP):break;
    case (GO_UP_START): 
        Res=ElevatorGoUpStop();
        if(Res!=0){printf("stop,GO_UP_START ERROR\r\n"); return 3;}
        current_operation=GO_UP_STOP;
        break;
    case (GO_UP_STOP):break;
    default:break;
  }
  
  return 0;
}

int GotoSetPosition(int SetHeight)
{
  if(SetHeight>MAX_LIFT){
    printf("Heght>MAX_LIFT\r\n");
    return 1;
  }
  int Diff=current_position-SetHeight;
  short ms;
  if(Diff>=DEAD_LENTH){
     ms=1.0*Diff/MAX_LIFT*LFIT_TIME;
     ElevatorGoDown(ms);
     current_position=SetHeight;
    //Down
  }
  else if(Diff<=-DEAD_LENTH){
     ms=-1.0*Diff/MAX_LIFT*LFIT_TIME;
     ElevatorGoUp(ms);
     current_position=SetHeight;
  }

}


void ElevatorInitial(void)
{
  com_initial();
  delay_ms(200);
  ElevatorGoUpStop();
  delay_ms(200);
  ElevatorGoDownStop();
  delay_ms(200);
  
  printf("ElevatorInitial,接下来下降\r\n");
  ElevatorGoDown(LFIT_TIME);
  delay_ms(200);
  printf("ElevatorInitial,下降结束\r\n");
  
  FlushCOMBuf();
}


int ElevatorTest(void)
{
  //ElevatorInitial();

  printf("ElevatorTest,接下来上升20\r\n");
				//GotoSetPosition(20);
				//delay_ms(200);
  ElevatorGoUp((float)20.0/MAX_LIFT*LFIT_TIME);
  delay_ms(250);	
  printf("ElevatorTest,上升结束\r\n");
  
CameraTest(0);

  printf("ElevatorTest,接下来下降20\r\n");
						//GotoSetPosition(0);
  ElevatorGoDown((float)20.0/MAX_LIFT*LFIT_TIME);
  delay_ms(250);
  printf("ElevatorTest,下降结束\r\n");
  
  FlushCOMBuf();
  return 0;
}
//*********************************************************************************************************************************************************************************//
char *CommandList[]={COMMAND_UP_STEP,COMMAND_DOWN_STEP,COMMAND_LEFT_STEP,COMMAND_RIGHT_STEP};
float CurrentAngleX=INI_ANGLE_X;
float CurrentAngleY=INI_ANGLE_Y;

void SendCommand(enumCommandTypeDef eCommand)
{
  system(  CommandList[(unsigned char)eCommand] );
}



void GoSteps(enumCommandTypeDef eCommand,int times)
{
  while(times--){
    system(  CommandList[(unsigned char)eCommand] );
	switch(eCommand)
	{
		case e_Up:   CurrentAngleY+=ANGLE_Y_STEP;break;
		case e_Down: CurrentAngleY-=ANGLE_Y_STEP;break;
		case e_Right: CurrentAngleX+=ANGLE_X_STEP;break;
		case e_Left:CurrentAngleX-=ANGLE_X_STEP;break;
		default :break;
	}
  }

  
}

int SetAngle(char AngleType,float Angle)
{
	
	float Temp=Angle-CurrentAngleX;
	unsigned short i;
	if(0==AngleType){  //  X   顺时针为正
		if(0>Angle  ||  360<Angle){
			return -1;
		}
	
		if(Temp>180){  
			i=Temp/ANGLE_X_STEP;
			GoSteps(e_Left,i );
			
		}
		else if(Temp<-180){
			i=-Temp/ANGLE_X_STEP;
			GoSteps(e_Right,i );
			
		}
		else if(Temp>0){
			i=Temp/ANGLE_X_STEP;
			GoSteps(e_Right,i );
			
		}
		else if(Temp<0){
			i=-Temp/ANGLE_X_STEP;
			GoSteps(e_Left,i );
			
		}
	}
	else if(1==AngleType){  //  Y   抬头为正
		if(-90>Angle  ||  90<Angle){
			return -1;
		}
	
		if(Temp>0){  
			i=Temp/ANGLE_X_STEP;
			GoSteps(e_Up,i );
			
		}
		else {
			i=-Temp/ANGLE_X_STEP;
			GoSteps(e_Down,i );
		}
	
	}
	CurrentAngleX=Angle;
	return 0;
}


int CameraTest(char *argv)
{
  unsigned short times=1;

 // printf("Camera,App Start\r\n\r\n");
  
//   printf("参数：%s\r\n\r\n",argv);
//   times=atoi(argv);
 //   printf("times==%d\r\n",times);

  GoSteps( e_Left,5);

 GoSteps( e_Up,3);
 GoSteps( e_Down,3);

  GoSteps( e_Right,5);
  return 0;
}


/*************************

int main(void)
{
  
  ElevatorTest();  

  CameraTest("9");

  return 0;
}
*/
