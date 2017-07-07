/*!
*     COPYRIGHT NOTICE
*     Copyright (c) 2013,山外科技
*     All rights reserved.
*     技术讨论：山外论坛 http://www.vcan123.com
*
*     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
*     修改内容时必须保留山外科技的版权声明。
*
* @file       main.c
* @brief      山外K60 平台主程序
* @author     山外科技
* @version    v5.0
* @date       2013-08-28
*/

#include "common.h"
#include "include.h"
#include "handle.h"
uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
uint8 stopline_num = 0;                         ////第一次检测到起跑线不算，第二次才是停车
//uint8 img[CAMERA_W*CAMERA_H];

//函数声明
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void zet_motor(void);
void steer(void);
void zet_oled();
void zet_camera();
//void PIT0_IRQHandler(void);
void zf_oled(int16 val);
void PIT1_IRQHandler();
void wzt_bluetooth(void);
uint16 speed_rember_R[3] = {0};
uint16 speed_rember_L[3] = {0};
//char * int_to_char(int a);
//void img_extract(uint8 *dst, uint8 *src, uint32 srclen);

/*!
*  @brief      main函数
*  @since      v5.3
*  @note       山外摄像头 LCD 测试实验

*/


void  main(void)
{
  //zet_bluetooth();
  uint16 send_data[4] = {0};
  uint8 time1=0;
  sum_time = 0;
  char nrf_data=0;
  uint8 Edge_R[3]= {0};
  uint8 Edge_L[3]= {0};
  uint8 IR1_last = 0;
  uint8 IR2_last = 0;
  
  //uint32 time2 =  0;
  DisableInterrupts;
  NVIC_SetPriority(PORTA_IRQn,1);
  NVIC_SetPriority(DMA0_IRQn,2);
  
  Init_All();
  DELAY_MS(1000);
  
  EnableInterrupts;
  
  set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);  
  set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);    
  int a=nrf_link_check(); 
  //uint8 Chaoche_stop_time=0;
  uint8 Chaoche_start_time=0;
  while(a)
  {
    pit_time_start(PIT1);
    camera_get_img();                                   //摄像头获取图像
    img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
    Search_Line();
    Find_Middle();
    Road_Type();
    Servo_control();
    
    speed_get_L = abs(ftm_quad_get(FTM1));          //获取FTM 正交解码 的脉冲数(负数表示反方向)
    if(Edge_L[0]!=0){
      if(abs(speed_get_L-speed_rember_L[2])<20)
      {
        speed_rember_L[0] = speed_rember_L[1];
        speed_rember_L[1] = speed_rember_L[2];
        speed_rember_L[2] = speed_get_L;
        
        Edge_L[0]=speed_rember_L[0];
        Edge_L[1]=speed_rember_L[1];
        Edge_L[2]=speed_rember_L[2];          
        speed_get_L=GetMedianNum(Edge_L,3);////左编码器滤波
      }
      else
      {
        speed_get_L = speed_rember_L[2];
      } 
      
    }
    speed_get_R = lptmr_pulse_get();
    if(Edge_R[0]!=0)
    {
      if(abs(speed_get_R-speed_rember_R[2])<20)
      {
        speed_rember_R[0] = speed_rember_R[1];
        speed_rember_R[1] = speed_rember_R[2];
        speed_rember_R[2] = speed_get_R;
        
        Edge_R[0]=speed_rember_R[0];
        Edge_R[1]=speed_rember_R[1];
        Edge_R[2]=speed_rember_R[2];          
        speed_get_R=GetMedianNum(Edge_R,3);////右编码器滤波          
      }
      else
      {
        speed_get_R = speed_rember_R[2];
      }
      
    }
    ftm_quad_clean(FTM1);
    lptmr_pulse_clean();
        
    if(stop_Flag !=1&&ChaoChe_stop!=1)//超车的时候电机不输出
    {
      Motor_Out();
    }

    if((Stop_Flag>1)&&Car==1&&ChaoChe_stop<2)
      stop_Car1();
    else if((Stop_Flag>1)&&Car==2&&stopLine_temp==0)
      stop_Car2();
    
     
    if(Stop_Flag==2&&stopLine_temp==1)//或者前车告诉后车有起跑线
    {
      if(Car==1)
        Servomiddle=8550;
//      else
//        Servomiddle=8700;
    }
    
    
    
    /*if(Stop_Flag==1&&sum_time>2000)
    {
      if(Car==1&&Cross_Flag!=Cross_Flag_Last&&Cross_Flag_Last==3)
      {
        ChaoChe_stop=1;
        
        ChaoChe_stop_time=0;
        gpio_set(PTE25,0);//停车，关闭前车超声波
        gpio_set(PTE24,0);
        race[0]=1;//race[0]通讯位，告诉后车，准备超车，让前车打开超声波
        ABDistance_last=2000;
      }
    }
    if(ChaoChe_stop==1)
    {
      Chaoche_stop();
    }
    
    if(ChaoChe_stop==1&&ABDistance>30)//超车后的后车（以前的前车）检测到超声波，
    {
      ChaoChe_stop=0;
      Car=2;
      race[1]=1;//race[1]通讯位，告诉前车，超车成功
      
      gpio_set(PTC3,1);
      gpio_set(PTC2,0);
      gpio_set(PTB17,0);
      gpio_set(PTB16,1);
      ftm_pwm_duty(FTM2,FTM_CH0,7000);//B2左电机
      ftm_pwm_duty(FTM2,FTM_CH1,7000);//B1右电机
    }
    
    if(ABDistance>300)
    {
      Car=2;
      race[1]=1;
    }
    */
    
 
    ///蓝牙传送编码器的值
    send_data[0] = Cross_Flag*500;
    send_data[1] = speed_PWM_L;
    send_data[2] = 0;
    send_data[3] = 0;

    //if(speed_get_R>50&&Cross_Flag!=0)
    //uart_putchar(UART5,Cross_Flag);
    vcan_sendware((uint16_t *)send_data, sizeof(send_data));
   
    nrf_rx(buff,4);               //等待接收一个数据包，数据存储在buff里
    
    uint8 nrf_data=0;
    for(int i=0;i<sizeof(buff);i++)
    {
      nrf_data|=buff[i];
      nrf_data=nrf_data<<1;
    }
    
    
    ////////////////后车检测到超声波信号，buff[1]发来一个1，表明超车成功
    /*if(buff[1]==1)
    {
      Car=1;
      race[0]=0;
      race[1]=0;
      ABDistance=0;
      ABDistance_last=0;
    }
    if(buff[0]==1)//前车告诉后车已经停车了buff[0]发来一个1
    {
      gpio_set(PTE25,1);//后车开启超声波
      gpio_set(PTE24,1);
    }
    */
    //if(Car==2)
    //race[0]=1;
    //Overtake_judge();

    if(speed_get_R<60&&speed_get_L<60)
    {
      dis_bmp(CAMERA_H,CAMERA_W,(uint8*)img,0x7F); 
      
      OLED_Print_Num1(88, 1, All_Black);
      OLED_Print_Num1(88, 2, error);
      OLED_Print_Num1(88, 3, errorerror);
      OLED_Print_Num1(88, 4, Kp);
      OLED_Print_Num1(88, 5, Kd);
      time1 = pit_time_get(PIT1)*1000/(bus_clk_khz*1000);
      //wzt_bluetooth();     
      OLED_Print_Num1(88, 6, Cross_Flag);
    }
    
    if(Stop_Flag==1&&speed_get_R!=0&&speed_get_L!=0)
    {
      sum_time++; 
    }
    pit_close(PIT1);
    //nrf_data = race[1];
    /*uart_putchar   (UART5 , Cross_Flag);
    uart_putchar   (UART5 , Right_xian);
    uart_putchar   (UART5 , Left_xian);
    uart_putchar   (UART5 , ring_num);*/
    //OLED_Print_Num1(88, 6, nrf_data);
  }
}

/*!
*  @brief      PORTA中断服务函数
*  @since      v5.0
*/
void PORTA_IRQHandler()
{
  uint8  n;    //引脚号
  uint32 flag;
  
  while(!PORTA_ISFR);
  flag = PORTA_ISFR;
  PORTA_ISFR  = ~0;                                   //清中断标志位
  
  n = 25;                                             //场中断
  if(flag & (1 << n))                                 //PTA29触发中断
  {
    camera_vsync();
  }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
  n = 28;
  if(flag & (1 << n))                                 //PTA28触发中断
  {
    camera_href();
  }
#endif
  
  
}

/*!
*  @brief      DMA0中断服务函数
*  @since      v5.0
*/
void DMA0_IRQHandler()
{
  camera_dma();
}

void zhidaochaoche(){
  //复制到main函数里
  /*   if(Car==1)
    {
      if(sum_time>0&&sum_time<100&&stopLine_temp==0&&Chaoche_start_time==0)
      {        
        if(Cross_Flag!=5&&Cross_Flag!=6&&speed_get_R!=0&&speed_get_L!=0)
        {
          Chaoche_stop();
          ChaoChe_stop_time++;
          //stop_Flag=0;
          //Chaoche_start();
        }
      }
    }
    else if(Car==2)
    {
      if(sum_time>0&&sum_time<100&&stopLine_temp==0&&Chaoche_start_time==0)
      {
        if(Cross_Flag!=5&&Cross_Flag!=6&&speed_get_R!=0&&speed_get_L!=0)
        {
          Chaoche_stop();
          ChaoChe_stop_time++;
          //stop_Flag=0;
          
        }
      }
    }
    
    if(ChaoChe_stop_time>0&&ChaoChe_stop_time<50)
      ChaoChe_stop_time++;
    else if(ChaoChe_stop_time>=50)
    {
      ChaoChe_stop_time=0;
      stop_Flag=0;
      Chaoche_start();
      Chaoche_start_time++;
    }
    
    if(Chaoche_start_time>0&&Chaoche_start_time<25)
      Chaoche_start_time++;
    else if(Chaoche_start_time>=25)    {
      Chaoche_start_time=0;
    }
    if(sum_time>0&&sum_time<100)
    {
      ChaoChe_temp=1;
      if(Car==1)
      {
        Servomiddle=8750;
      }
      else if(Car==2)
        Servomiddle=8560;
    }
    else
    {
      ChaoChe_temp=0;
    }
*/
}

