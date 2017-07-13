#include "common.h"
#include "include.h"
#include "function.h"

#include "handle.h"

uint16 speed_goal_R;
uint16 speed_goal_L;
uint16 speed_goal;
//uint16 speed_get;
uint16 speed_get_R=0;
uint16 speed_get_L=0;
uint16 speed_last=0;
uint16 speed_nexterr=0;
uint16 speed_lasterr=0;
uint16 speed_PWM=0;
int16 speed_PWM_R = 0;
int16 speed_PWM_L = 0;
int16  speed_err_R = 0;
int16  speed_err_L = 0;
int16 speed_err_R_last = 0;
int16 speed_err_L_last = 0;
int16 speed_err_R_lastlast = 0;
int16 speed_err_L_lastlast = 0;
int16 speed_increment_R;
int16 speed_increment_L;
uint8 Status=0;
uint16 var;
uint8 stop_Flag = 0;
uint8 stop_Place = 0;
uint32 Distance = 900;
uint8 stop_time = 0;
uint8 Car_First_stop=0;
uint8 Car_Second_stop=0;
uint8 ChaoChe_stop_time = 0;
uint8 ChaoChe_Cross_temp = 0;
uint8 Distance_stop_temp=0;
/*uint8 Edge_R[3]= {0};
uint8 Edge_L[3]= {0};   
uint8 stopline_num = 0;
uint16 speed_rember_R[3] = {0};
uint16 speed_rember_L[3] = {0};
*/
void xx_bluetooth();
void uart3_handler(void);
extern uint8 imgbuff[CAMERA_SIZE];
void PORTC_IRQHandler();
void xx_bluetooth();
void uart5_handler(void);
void chaoShenBo_init(void);
void stopLine_init(void);
void Switch_Init();
uint8 Get_Switch(void);
void PIT0_IRQHandler(void)
{
  if(Cross_Flag==3&&stop_Flag !=1&&stop_Place==1)
  {
    //stop_Car();
  }
  //ftm_quad_clean(FTM1);
  // Search_Line();
  //Find_Middle();
  //Servo_control();
  //zf_oled( val);

  PIT_Flag_Clear(PIT0);       //清中断标志位
}



void Init_All(void)
{
  Car=1;
  Motor_Init();
  OLED_Init();
  ov7725_eagle_init(imgbuff);
  xx_bluetooth();
  ftm_quad_init(FTM1);
  lptmr_pulse_init(LPT0_ALT2,0xFFFF,LPT_Rising);
  pit_init_ms(PIT0, 50);                             //初始化PIT0，定时时间为： 1000ms       
  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);       //设置PIT0的中断服务函数为 PIT_IRQHandler
  stopLine_init();
  Switch_Init();
  while(!nrf_init());
  set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);                //设置 PORTE 的中断服务函数为 PORTE_VECTORn
  chaoShenBo_init();
  Status=Get_Switch();
  enable_irq(PORTC_IRQn);
  enable_irq (PIT0_IRQn);                                //使能PIT0中断
}


/*
*       电机舵机初始化
*
*/
void Motor_Init(void)
{
    ftm_pwm_init(FTM2,FTM_CH1,15000,0);//驱动B1FTM初始化
    gpio_init(PTC3,GPO,0);//驱动正向使能初始化
    gpio_init(PTC2,GPO,0);//驱动反向使能初始化
    gpio_set(PTC3,1);
    gpio_set(PTC2,0);
    
    ftm_pwm_init(FTM2,FTM_CH0,15000,0);//驱动B2FTM初始化
    gpio_init(PTB17,GPO,0);//驱动正向使能初始化
    gpio_init(PTB16,GPO,0);//驱动反向使能初始化
    gpio_set(PTB17,0);
    gpio_set(PTB16,1);
    
    ftm_pwm_init(FTM0,FTM_CH3,100,0);
    //DELAY_MS(10);
    
    ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle);
    
}
/*
*       电机PID输出
*
*/
void Motor_Out(void)
{
  
  speed_PWM=6550;
  uint8 speed_Ki=5;
  uint8 speed_Kp=20;  
  uint8 speed_Kd=8;
  gpio_set(PTC3,1);
  gpio_set(PTC2,0);
  gpio_set(PTB17,0);
  gpio_set(PTB16,1);
  
  if(All_Black>=50)
  {
    speed_PWM_R=0;
    speed_PWM_L =0;
    
  }
  else
  { 
    if(stop_time==0)
    {      
      if(abs(error)<4)//直道
      {
        if(Status==0)
        {
          speed_goal=3800;
        }
        else if(Status==1)
        {
          speed_goal=4100;
        }
        else if(Status==2)//直道加速
        {
          speed_goal=4600;
        }
        
        else if(Status==3)
        {
          speed_goal=5000;
        }
        
         else if(Status==4)
        {
          speed_goal=5200;
        }
      }
      
      else//弯道速度
      {
       if(Status==0)
        {
          speed_goal=3800;
        }
        else if(Status==1)
        {
          speed_goal=4100;
        }
        else if(Status==2)//入弯减速
        {
          speed_goal=4300;
        }
        
        else if(Status==3)
        {
          speed_goal=4800;
        }
       
       else if(Status==4)
        {
          speed_goal=5000;
        }
      }     
     
      if(Car==2)
      {
        if(ABDistance<Distance-50)
        {
          speed_goal = speed_goal-400;
        }

        else if(ABDistance>Distance+50)
        {
          speed_goal = speed_goal+400;
        }
      }     
      speed_goal_R=speed_goal;
      speed_goal_L=speed_goal;
      
//      if((abs(error)<8&&abs(error)>=4)||(All_Black>4&&All_Black<8)||Cross_Flag==3)
//      {
////        if(abs(error)<8&&abs(error)>=4)
////          stop1();
////        else 
//          if((All_Black>2&&All_Black<15)||Cross_Flag==31)
//        {
//          stop1();
//        }
//          
//        return;
//      }
      speed_err_R_lastlast = speed_err_R_last;
      speed_err_R_last = speed_err_R;
      
      speed_err_L_lastlast = speed_err_L_last;
      speed_err_L_last = speed_err_L;
      
      speed_err_R = speed_goal_R-speed_get_R*10;
      speed_err_L = speed_goal_L-speed_get_L*10;
      
      speed_increment_R = speed_Kp*(speed_err_R-speed_err_R_last)/10+
                              speed_Ki*speed_err_R/10+
                            speed_Kd*(speed_err_R-2*speed_err_R_last+speed_err_R_lastlast)/10;
      speed_increment_L= speed_Kp*(speed_err_L-speed_err_L_last)/10+
                          speed_Ki*speed_err_L/10+
                            speed_Kd*(speed_err_L-2*speed_err_L_last+speed_err_L_lastlast)/10;
      speed_PWM_R=6100+speed_increment_R;
      speed_PWM_L=6100+speed_increment_L;
      
    }
    else if(stop_time<3)
    {
      stop_time++;
    }
    else
      stop_time=0;
  }
  
//  if((speed_get_L<50||speed_get_R<50)&&Stop_Flag!=0&&sum_time>10)
//  {
//    if(speed_get_L<50)
//    { 
//      speed_PWM_L = 0;
//    }
//    if(speed_get_R<50)
//    {
//      speed_PWM_R = 0;
//    }
//  }
//  else 
    if(speed_get_R<100||speed_get_L<100)
  {
    if(speed_get_R<100)
      speed_PWM_R = 6800;
    if(speed_get_L<100)
      speed_PWM_L = 6800;
  }
  
 if(speed_PWM_R<0)
  {
    gpio_set(PTC3,0);//驱动反向使能
    gpio_set(PTC2,1);//驱动反向使能
    gpio_set(PTB17,1);//驱动反向使能
    gpio_set(PTB16,0);//驱动反向使能
    speed_PWM_R=abs(speed_PWM_R);
    
  }
  if(speed_PWM_R>8800)
    speed_PWM_R=8800;
  
  if(speed_PWM_L<0)
  {
    gpio_set(PTC3,0);//驱动反向使能
    gpio_set(PTC2,1);//驱动反向使能
    gpio_set(PTB17,1);//驱动反向使能
    gpio_set(PTB16,0);//驱动反向使能
    speed_PWM_L = abs(speed_PWM_L);
  }
  if(speed_PWM_L>8800)
    speed_PWM_L=8800;
   
  ftm_pwm_duty(FTM2,FTM_CH0,speed_PWM_L);//B2左电机
  ftm_pwm_duty(FTM2,FTM_CH1,speed_PWM_R);//B1右电机
  
}
/*
*点刹
*/
void stop1(void)
{
  //gpio_set(PTC3,0);//驱动反向使能
  //gpio_set(PTC2,1);//驱动反向使能
  //gpio_set(PTB17,1);//驱动反向使能
  //gpio_set(PTB16,0);//驱动反向使能
  ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,0);//B1
  stop_time++;
  //DELAY_MS(10);
  gpio_set(PTC3,1);//驱动反向使能
  gpio_set(PTC2,0);//驱动反向使能
  gpio_set(PTB17,0);//驱动反向使能
  gpio_set(PTB16,1);//驱动反向使能
}
  
/*
*点反刹
*/
void stop2(void)
{
   gpio_set(PTC3,0);//驱动反向使能
  gpio_set(PTC2,1);//驱动反向使能
  gpio_set(PTB17,1);//驱动反向使能
  gpio_set(PTB16,0);//驱动反向使能
  ftm_pwm_duty(FTM2,FTM_CH0,8800);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,8800);//B1
  stop_time++;
}
/*
*2车停车
*/
void stop_Car2(void)
{ 
  gpio_set(PTC3,0);//驱动反向使能
  gpio_set(PTC2,1);//驱动反向使能
  gpio_set(PTB17,1);//驱动反向使能
  gpio_set(PTB16,0);//驱动反向使能
  ftm_pwm_duty(FTM2,FTM_CH0,9500);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,9500);//B1
  ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle);
  DELAY_MS(150);
  ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,0);//B1
  
  stop_Flag  = 1;
  Car_Second_stop=1;
}
/*
*1车停车
*/
void stop_Car1()
{
  if(speed_get_L<8&&speed_get_R<8)
  {
    ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,0);//B1  
    //stop_Flag  = 1;
    DELAY_MS(1000);//暂时这样用，等以后用NRF通知后车接近再发车
    
    gpio_set(PTC3,1);
    gpio_set(PTC2,0);
    gpio_set(PTB17,0);
    gpio_set(PTB16,1);
    ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle);
    ftm_pwm_duty(FTM2,FTM_CH0,6800);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,6800);//B1
    DELAY_MS(800);
    
    gpio_set(PTC3,0);//驱动反向使能
    gpio_set(PTC2,1);//驱动反向使能
    gpio_set(PTB17,1);//驱动反向使能
    gpio_set(PTB16,0);//驱动反向使能
    ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,0);//B1
    Car_First_stop=2;
    stop_Flag  = 1;
    return;
  }
  
  stop();
}

/*
后车检测到小于一定距离就停车
*/
void Distance_stop(void)
{
  if(speed_get_L<8&&speed_get_R<8)
  {
    gpio_set(PTC3,0);//驱动反向使能
    gpio_set(PTC2,1);//驱动反向使能
    gpio_set(PTB17,1);//驱动反向使能
    gpio_set(PTB16,0);//驱动反向使能
    ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,0);//B1
    Distance_stop_temp=1;//防止多次停车
    return;
  }
  stop();
}


void stop(void){
  gpio_set(PTC3,1);
  gpio_set(PTC2,0);
  gpio_set(PTB17,0);
  gpio_set(PTB16,1);
  
  
  uint8 speed_Ki=10;
  uint8 speed_Kp=20;  
  uint8 speed_Kd=5;
  
  if(All_Black>=50)
  {
    speed_PWM_R=0;
    speed_PWM_L =0;
    
  }
  else
  { 
      speed_goal_R=0;//设置速度
      speed_goal_L=0;
  }
  speed_err_R_lastlast = speed_err_R_last;
  speed_err_R_last = speed_err_R;
  
  speed_err_L_lastlast = speed_err_L_last;
  speed_err_L_last = speed_err_L;
  
  speed_err_R = speed_goal_R-speed_get_R*10;
  speed_err_L = speed_goal_L-speed_get_L*10;
  
  speed_increment_R = speed_Kp*(speed_err_R-speed_err_R_last)/10+
    speed_Ki*speed_err_R/10+
      speed_Kd*(speed_err_R-2*speed_err_R_last+speed_err_R_lastlast)/10;
  speed_increment_L= speed_Kp*(speed_err_L-speed_err_L_last)/10+
    speed_Ki*speed_err_L/10+
      speed_Kd*(speed_err_L-2*speed_err_L_last+speed_err_L_lastlast)/10;
  speed_PWM_R=6100+speed_increment_R;
  speed_PWM_L=6100+speed_increment_L;
  
  
  
  if(speed_PWM_R<0)
  {
    gpio_set(PTC3,0);//驱动反向使能
    gpio_set(PTC2,1);//驱动反向使能
    gpio_set(PTB17,1);//驱动反向使能
    gpio_set(PTB16,0);//驱动反向使能
    speed_PWM_R=abs(speed_PWM_R);
    
  }
  if(speed_PWM_R>8800)
    speed_PWM_R=8800;
  
  if(speed_PWM_L<0)
  {
    gpio_set(PTC3,0);//驱动反向使能
    gpio_set(PTC2,1);//驱动反向使能
    gpio_set(PTB17,1);//驱动反向使能
    gpio_set(PTB16,0);//驱动反向使能
    speed_PWM_L = abs(speed_PWM_L);
  }
  if(speed_PWM_L>8800)
    speed_PWM_L=8800;
  
  ftm_pwm_duty(FTM2,FTM_CH0,speed_PWM_L);//B2左电机
  ftm_pwm_duty(FTM2,FTM_CH1,speed_PWM_R);//B1右电机
}


void get_error()
{
  uint8 Lastline=0;
  uint8 Row_Ptr=0;
  uint8 l=0;
  if(All_Black>2)
  {
    Lastline=All_Black;
  }
  else
  {
    Lastline=2;
  }
  
  l = 59-Lastline;
  
  for(Row_Ptr=59; Row_Ptr>Lastline; Row_Ptr--)
  {
    error+=(Road_Center[Row_Ptr]-40);
  }
  error = error/l;
  for(Row_Ptr=59; Row_Ptr>59-l/2; Row_Ptr--)
  {
    error1+=(Road_Center[Row_Ptr]-40);
  }
  for(Row_Ptr=59-l/2; Row_Ptr>Lastline; Row_Ptr--)
  {
    error2+=(Road_Center[Row_Ptr]-40);
  }
  error1 = error1*2/l;
  error2 = error2/(59-l/2-Lastline);
  errorerror = error2-error1;
}
/*
超车
*/
void Chaoche_FrontCar(void)
{
  uint8 All_black_rember=All_Black;
  int16 error_rember=error;
  int16 errorerror_rember=errorerror;
  
  ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle-150);
  
  gpio_set(PTC3,0);//驱动反向使能
  gpio_set(PTC2,1);//驱动反向使能
  gpio_set(PTB17,1);//驱动反向使能
  gpio_set(PTB16,0);//驱动反向使能
  ftm_pwm_duty(FTM2,FTM_CH0,7000);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,7000);//B1
  //ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle);
  DELAY_MS(150);
  
  gpio_set(PTC3,1);
  gpio_set(PTC2,0);
  gpio_set(PTB17,0);
  gpio_set(PTB16,1);
  //ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle);
  ftm_pwm_duty(FTM2,FTM_CH0,7000);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,7000);//B1
  DELAY_MS(150);
  gpio_set(PTC3,0);//驱动反向使能
  gpio_set(PTC2,1);//驱动反向使能
  gpio_set(PTB17,1);//驱动反向使能
  gpio_set(PTB16,0);//驱动反向使能
  ftm_pwm_duty(FTM2,FTM_CH0,7000);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,7000);//B1
  DELAY_MS(180);
  
  ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,0);//B
  
  NRF_SendData(10002);//告诉后车有十字路口
  
  DELAY_MS(400);
  
  Car=2;
  gpio_set(PTE25,0);//关闭超声波
  gpio_set(PTE24,0);
  Overtake++;
  uint8 time=0;
  do
  {
    
    error=0;
    errorerror=0;
    Cross_Flag=0;
    gpio_set(PTC3,0);//驱动反向使能
    gpio_set(PTC2,1);//驱动反向使能
    gpio_set(PTB17,1);//驱动反向使能
    gpio_set(PTB16,0);//驱动反向使能
    ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle-100);
    ftm_pwm_duty(FTM2,FTM_CH0,7000);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,7000);//B1
    
    camera_get_img();                                   //摄像头获取图像
    img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
    Search_Line();
    Find_Middle();
    
//    speed_get_L = abs(ftm_quad_get(FTM1));
//    speed_get_R = lptmr_pulse_get();
//    ftm_quad_clean(FTM1);
//    lptmr_pulse_clean();
//    if(speed_get_R!=0&&speed_get_L!=0)
//    {
      time++;
//    }
    if(time>1)
    {
      Servo_control();
    }
    else
    {
      get_error();
      ftm_pwm_duty(FTM0, FTM_CH3, Servo_min);
      DELAY_MS(600);
      Cross_Flag=0;
    }
    
//    if(speed_get_R<60&&speed_get_L<60)
//    {
//      dis_bmp(CAMERA_H,CAMERA_W,(uint8*)img,0x7F); 
//      OLED_Print_Num1(88, 1, time);
//      OLED_Print_Num1(88, 2, error);
//      OLED_Print_Num1(88, 3, errorerror);
//      OLED_Print_Num1(88, 4, error_rember);
//      OLED_Print_Num1(88, 5, errorerror_rember);
//      
//      OLED_Print_Num1(88, 6, Cross_Flag);
//    }
  }while(Cross_Flag!=1&&time<500&&(abs(errorerror-errorerror_rember)>4||abs(error-error_rember)>4));
  
  do
  {
    gpio_set(PTC3,0);//驱动反向使能
    gpio_set(PTC2,1);//驱动反向使能
    gpio_set(PTB17,1);//驱动反向使能
    gpio_set(PTB16,0);//驱动反向使能
    ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle-150);
    ftm_pwm_duty(FTM2,FTM_CH0,7000);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,7000);//B1
    
    camera_get_img();                                   //摄像头获取图像
    img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
    Search_Line();
    Find_Middle();
    Servo_control();
//    speed_get_L = abs(ftm_quad_get(FTM1));
//    speed_get_R = lptmr_pulse_get();
//    ftm_quad_clean(FTM1);
//    lptmr_pulse_clean();
//    if(speed_get_R<60&&speed_get_L<60)
//    {
//      dis_bmp(CAMERA_H,CAMERA_W,(uint8*)img,0x7F); 
//      OLED_Print_Num1(88, 1, error);
//      OLED_Print_Num1(88, 2, error_rember);
//      OLED_Print_Num1(88, 3, errorerror);
//      OLED_Print_Num1(88, 4, errorerror_rember);
//      OLED_Print_Num1(88, 5, Left_stop);
//      
//      OLED_Print_Num1(88, 6, Right_stop);
//    }
  }while(Left_stop>35||Right_stop>35);
  
  gpio_set(PTC3,1);
  gpio_set(PTC2,0);
  gpio_set(PTB17,0);
  gpio_set(PTB16,1);
  camera_get_img();                                   //摄像头获取图像
  img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
  Search_Line();
  Find_Middle();
  Servo_control();
  ftm_pwm_duty(FTM2,FTM_CH0,9500);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,9500);//B1
  NRF_SendData(10001);//告诉2车超车成功
  DELAY_MS(300);
  //ChaoChe_temp=0;
}


//NRF
void NRF_SendData(int data)
{
  uint8 sendData[4]={0};
  for(uint8 i=0;i<4;i++)
  {
    sendData[i] = data%10;
    data = data/10;
  }
  nrf_tx(sendData,4);
  while(nrf_tx_state() == NRF_TXING);//等待发送完成
  
}




//拨码开关初始化

void Switch_Init()
{
  	
	gpio_init(PTE4,GPI,0);
	gpio_init(PTE3,GPI,0);
	gpio_init(PTE2,GPI,0);
	gpio_init(PTE1,GPI,0);
        gpio_init(PTE0,GPI,0);
	
	port_init_NoALT(PTE4,PULLUP);
	port_init_NoALT(PTE3,PULLUP);
	port_init_NoALT(PTE2,PULLUP);
	port_init_NoALT(PTE1,PULLUP);
        port_init_NoALT(PTE0,PULLUP);
}
//获取拨码开关值
uint8 Get_Switch(void)
{
  uint8 Num=0;
  Num|=gpio_get(PTE4);
  Num=Num<<1;
  Num|=gpio_get(PTE3);
  Num=Num<<1;
  Num|=gpio_get(PTE2);
  Num=Num<<1;
  Num|=gpio_get(PTE1);
  Num=Num<<1;
  Num|=gpio_get(PTE0);
  return Num;
}
/*
* 蓝牙初始化
*
*/
void xx_bluetooth()
{
  uart_init(UART5,9600);     //初始化串口(UART3 是工程里配置为printf函数输出端口，故已经进行初始化)
  //uart_putstr   (UART5 ,"\n\n\n接收中断测试：");           //发送字符串
  set_vector_handler(UART5_RX_TX_VECTORn,uart5_handler);   // 设置中断服务函数到中断向量表里
  uart_rx_irq_en (UART5);                                 //开串口接收中断
}

/*
* 蓝牙串口中断服务函数
*/
void uart5_handler(void)
{
  char ch;
  
  if(uart_query    (UART5) == 1)   //接收数据寄存器满
  {
    //用户需要处理接收数据
    uart_getchar   (UART5, &ch);                    //无限等待接受1个字节
    uart_putchar   (UART5 , ch);                    //发送字符串
    uart_putstr   (UART5 ,"\n\n\n接收中断测试：");
  }
}


/*
* NRF中断函数
*/
void PORTC_IRQHandler()
{
  uint8  n;    //引脚号
  uint32 flag;
  
  flag = PORTC_ISFR;
  PORTC_ISFR  = ~0;                                   //清中断标志位
  
  n = 0;
  if(flag & (1 << n))                                 //PTC0触发中断
  {
    nrf_handler();
  }
  
}


/*
*超声波的PIT初始化
*/
void ChaoShenBo_PitInit(PITn_e pitn)
{
  SIM_SCGC6   |= SIM_SCGC6_PIT_MASK;//module clock  
  PIT_MCR     &= ~PIT_MCR_MDIS_MASK;//pit module enable 
  PIT_LDVAL(pitn)   = 0xFFFFFFFF;          // 
  PIT_Flag_Clear(pitn);       //清中断标志位
  //   PIT_TCTRL0  |= PIT_TCTRL_TIE_MASK;//Enable interrupt
  //   PIT_TCTRL0  |= PIT_TCTRL_TEN_MASK;//enable the timer,run
}
/*
* 超声波初始化
*
*/
void chaoShenBo_init(void)
{
  gpio_init(PTE27,GPI,1);//使用GPIOA的中断，初始化IO口为输入，上拉
  port_init(PTE27, IRQ_EITHER | PF | ALT1 | PULLUP);
  if(Car==1)
  {
    gpio_init(PTE25,GPO,1);//前车发送
    gpio_init(PTE24,GPO,1);//前车发送
    
  }
  else
  {
    gpio_init(PTE25,GPO,0);
    gpio_init(PTE24,GPO,0);
  }
  NVIC_SetPriority(PORTE_IRQn,0);//超声波的中断，优先级必须设置为最高的
  set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);                //设置 PORTE 的中断服务函数为 PORTE_VECTORn
  
  enable_irq(PORTE_IRQn);
  ChaoShenBo_PitInit(PIT2);  
  NVIC_SetPriority(PIT2_IRQn,3);//这个中断无所谓,
}

void stopLine_init(void)
{
  gpio_init(PTE10,GPI,1);//使用GPIOE的中断，初始化IO口为输入，上拉
  //port_init(PTE10, IRQ_RISING | PF | ALT1 | PULLUP);
  gpio_init(PTE9,GPI,1);
  //port_init(PTE9, IRQ_RISING | PF | ALT1 | PULLUP);
}