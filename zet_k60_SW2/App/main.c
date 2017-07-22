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
  uint16 send_data[3] = {0};
  uint8 time1=0;
  sum_time = 0;
  char nrf_data=0;
  uint8 Edge_R[3]= {0};
  uint8 Edge_L[3]= {0};
  
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
  uint8 rember_time=0;
  uint32  ABDistance_rember=0;
  while(a)
  {
    pit_time_start(PIT1);
    camera_get_img();                                   //摄像头获取图像
    img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
    Search_Line();
    nrf_rx(buff,4);               //等待接收一个数据包，数据存储在buff里      
      
    if(Car==2)
    {
      
      if(buff[3]==0&&buff[2]==0&&buff[1]==0&&buff[0]==1)
      {
        Car=1;
        Overtake++;//超车次数加1
        ABDistance=0;
        ABDistance_last=0;
        gpio_set(PTE25,1);//后车开启超声波
        gpio_set(PTE24,1);
        ChaoChe_Cross_temp=0;
      }
      if(buff[3]==0&&buff[2]==0&&buff[1]==0&&buff[0]==3)
      {
        Car=1;
        Ring_OverTake++;
        ABDistance=0;
        ABDistance_last=0;
        gpio_set(PTE25,1);//后车开启超声波
        gpio_set(PTE24,1);
        ChaoChe_Cross_temp=0;
      }
      if(buff[3]==0&&buff[2]==0&&buff[1]==0&&buff[0]==2)
        ChaoChe_Cross_temp=1;
      
      if(ChaoChe_Cross_temp==1)
      {
        Cross_Flag=1;
      }
      
    }
    if(Car==1)
    {
      if(buff[3]==0&&buff[2]==0&&buff[1]==1&&buff[0]==1)//距离正常
      {
        Distance_temp=1;
        Distance_temp_rember=Distance_temp;
      }
      else if(buff[3]==0&&buff[2]==0&&buff[1]==1&&buff[0]==0)//距离过小
      {
        Distance_temp=0;
        Distance_temp_rember=Distance_temp;
      }
      else if(buff[3]==0&&buff[2]==0&&buff[1]==1&&buff[0]==2)//距离过大
      {
        Distance_temp=2;
        Distance_temp_rember=Distance_temp;
      }
      else if(buff[3]==0&&buff[2]==0&&buff[1]==1&&buff[0]==3)//距离无效
        Distance_temp=3;
      else
        Distance_temp=4;
    }
    
//    if(Right_stop>40)
//    {
//      if(Car==2&&ChaoChe_Cross_temp==1)
//      {
//        Car=1;
//        ABDistance=0;
//        ABDistance_last=0;
//        gpio_set(PTE25,1);//开启超声波
//        gpio_set(PTE24,1);
//        NRF_SendData(20001);//告诉停着的车，超车成功，换标志位
//        ChaoChe_Cross_temp=0;
//      }
//      
//    }
    
//    if(Car==1)
//    {
//      nrf_rx(buff,4);               //等待接收一个数据包，数据存储在buff里     
//      if(buff[3]==0&&buff[2]==0&&buff[1]==0&&buff[0]==1)
//      {
//        Car=2;
//        gpio_set(PTE25,0);//停着的车关闭超声波
//        gpio_set(PTE24,0);
//      }
//    }
    
    Find_Middle();
    Road_Type();
    Servo_control();
    
    speed_get_L = abs(ftm_quad_get(FTM1));          
    speed_get_R = lptmr_pulse_get();
    ftm_quad_clean(FTM1);
    lptmr_pulse_clean();
        
    if(stop_Flag !=1&&Car_First_stop==0&&Car_Second_stop==0)//停车的时候电机不输出
    {  
      if(Car==1)
        Motor_Out();
      else if(Car==2)
      {
//        if(ABDistance>1000||ABDistance<=10)
//        {
          Motor_Out();
//        }
//        else if(Distance_stop_temp==0&&ABDistance<=1000&&ABDistance>10)
//        {
//          Distance_stop();
//        }
      }
    }    
      if((Stop_Flag>1)&&Car==1&&stopLine_temp==0&&Car_First_stop<2&&stop_Flag==0)
        stop_Car2();
      else if((Stop_Flag>1)&&Car==2&&stopLine_temp==0&&Car_Second_stop==0&&stop_Flag==0)
        stop_Car2();
    
      if(gpio_get(PTE4)==1&&Distance_temp>1&&Cross_Flag==1&&((Left_stop>17&&Left_stop<22)||(Right_stop>17&&Right_stop<2))&&(Right_stop_find_temp==1||Left_stop_find_temp==1)&&Car==1&&Overtake<1)
      {
        Chaoche_FrontCar();
      }
      
<<<<<<< HEAD
    if(gpio_get(PTE2)==1/*&&Distance_temp<2*/&&Cross_Flag_Last==31&&Ring_First_Row>27&&Car==1&&Ring_OverTake==0)//距离控制标志位没有加
=======
    if(gpio_get(PTE2)==1/*&&Distance_temp<2*/&&Cross_Flag_Last==31&&Ring_First_Row>22&&Car==1&&Ring_OverTake==0)//距离控制标志位没有加
>>>>>>> 9b00e734f93fb78dedfb9b7469ff02f0cfac2959
    {
      Ring_Overtake();
      rember_time=1;
      
    }
    if(rember_time>0)
    {
      if(Ring_First_Row!=0)
      {
        camera_get_img();                                   //摄像头获取图像
        img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
        Search_Line();
        
        Find_Middle();
        Road_Type();
        Servo_control();
        if(gpio_get(PTE3)==1&&Ring_First_Row!=0)
        {          
<<<<<<< HEAD
          if(Servo_temp>55&&Servo_temp<110)
            ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle_Rember+Servo_temp);
          else
            ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle_Rember+105);
        }
        else
        {
          if(Servo_temp>55&&Servo_temp<110)
            ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle_Rember-Servo_temp);
          else
            ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle_Rember-105);
=======
          if(Servo_temp>55&&Servo_temp<105)
            ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle_Rember+Servo_temp);
          else
            ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle_Rember+100);
        }
        else
        {
          if(Servo_temp>55&&Servo_temp<105)
            ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle_Rember-Servo_temp);
          else
            ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle_Rember-100);
>>>>>>> 9b00e734f93fb78dedfb9b7469ff02f0cfac2959
        }
        gpio_set(PTC3,1);
        gpio_set(PTC2,0);
        gpio_set(PTB17,0);
        gpio_set(PTB16,1);
        //ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle);
        ftm_pwm_duty(FTM2,FTM_CH0,7500);//B2
        ftm_pwm_duty(FTM2,FTM_CH1,7500);//B1
        DELAY_MS(140);
        
        camera_get_img();                                   //摄像头获取图像
        img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
        Search_Line();
        
        Find_Middle();
        Road_Type();
        Servo_control();
        
        gpio_set(PTC3,0);//驱动反向使能
        gpio_set(PTC2,1);//驱动反向使能
        gpio_set(PTB17,1);//驱动反向使能
        gpio_set(PTB16,0);//驱动反向使能
        ftm_pwm_duty(FTM2,FTM_CH0,7000);//B2
        ftm_pwm_duty(FTM2,FTM_CH1,7000);//B1
        DELAY_MS(130);
        camera_get_img();                                   //摄像头获取图像
        img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
        Search_Line();
        Find_Middle();
        Road_Type();
        Servo_control();
          
        ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
        ftm_pwm_duty(FTM2,FTM_CH1,0);//B
        
      }
      if(speed_get_R<80&&speed_get_L<80)
      {
        uint8 wait_temp=0;
        uint8 t_t=0;
        do
        {
          
          camera_get_img();                                   //摄像头获取图像
          img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
          Search_Line();
          t_t++;
          Find_Middle();
          Road_Type();
          Servo_control();
          nrf_rx(buff,4);               //等待接收一个数据包，数据存储在buff里     
          if(buff[3]==0&&buff[2]==0&&buff[1]==1&&(buff[0]==1||buff[0]==0)||(Distance_temp_rember==0||

Distance_temp_rember==1))//距离正常或小于
            wait_temp=1;
          
<<<<<<< HEAD
        }while(wait_temp==0||t_t<40);
=======
        }while(wait_temp==0||t_t<45);
>>>>>>> 9b00e734f93fb78dedfb9b7469ff02f0cfac2959
//        if(t_t>=80)
//          DELAY_MS(300);
//        else
//          DELAY_MS(500);
        rember_time=0; 
        stop_Flag=0;
        Car=2;
<<<<<<< HEAD
        
=======
>>>>>>> 9b00e734f93fb78dedfb9b7469ff02f0cfac2959
        gpio_set(PTE25,0);//关闭超声波
        gpio_set(PTE24,0);
        uint8 t=0;
        camera_get_img();                                   //摄像头获取图像
        img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
        Search_Line();
        
        Find_Middle();
        Road_Type();
        Servo_control();
        gpio_set(PTC3,1);//驱动反向使能
        gpio_set(PTC2,0);//驱动反向使能
        gpio_set(PTB17,0);//驱动反向使能
        gpio_set(PTB16,1);//驱动反向使能
        t++;
        ftm_pwm_duty(FTM2,FTM_CH0,9500);//B2
        ftm_pwm_duty(FTM2,FTM_CH1,9500);//B1
        DELAY_MS(100);
        NRF_SendData(10003);//告诉后车圆环超车
      }
      else 
      {
        gpio_set(PTC3,0);//驱动反向使能
        gpio_set(PTC2,1);//驱动反向使能
        gpio_set(PTB17,1);//驱动反向使能
        gpio_set(PTB16,0);//驱动反向使能
        ftm_pwm_duty(FTM2,FTM_CH0,5500);//B2
        ftm_pwm_duty(FTM2,FTM_CH1,5500);//B1
        
      }
    }
    ///蓝牙传送编码器的值
    send_data[0] = 0;
    send_data[1] = Cross_Flag*500;
    send_data[2] = 0;
    //vcan_sendware((uint16_t *)send_data, sizeof(send_data));
    //Overtake_judge();
    if(speed_get_R<60&&speed_get_L<60)
    {
      dis_bmp(CAMERA_H,CAMERA_W,(uint8*)img,0x7F); 
        OLED_Print_Num1(88, 1, Car);
      OLED_Print_Num1(88, 2, error);
      OLED_Print_Num1(88, 3, errorerror);
      OLED_Print_Num1(88, 4, Kp);
      OLED_Print_Num1(88, 5, Kd);

      time1 = pit_time_get(PIT1)*1000/(bus_clk_khz*1000);   
      OLED_Print_Num1(88, 6, ABDistance);
    }
    
    if(Car==2)
    {
      if(ABDistance_rember!=ABDistance)
      {
        ABDistance_rember=ABDistance;
        if(abs(ABDistance-Distance)<150)
          NRF_SendData(20011);
        else if(ABDistance<Distance-150)
          NRF_SendData(20010);
        else 
          if(ABDistance>Distance+150)
            NRF_SendData(20012);
      }
      else
        NRF_SendData(20013);
    }
    if(Stop_Flag==1&&speed_get_R!=0&&speed_get_L!=0)
    {
      sum_time++;
//      if(sum_time-rember_time==5)
//      {
//      }
//      else if(sum_time-rember_time==6)
//        rember_time==sum_time;
    }
//    if(speed_get_R!=0&&speed_get_L!=0)
//    {
//      sum_time++; 
//    }
//
//    if(sum_time>250)
//      stop_Car2();
    pit_close(PIT1);
    nrf_data = race[1];
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


void zhidaochaoche(){
  /*if(Car==1)
    {
      if(sum_time>5&&sum_time<100&&stopLine_temp==0&&Chaoche_start_time==0)
      {
        if(Cross_Flag!=5&&Cross_Flag!=6&&speed_get_R>30&&speed_get_L>30)
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
      if(sum_time>15&&sum_time<100&&stopLine_temp==0&&Chaoche_start_time==0)
      {
        if(Cross_Flag!=5&&Cross_Flag!=6&&speed_get_R>30&&speed_get_L>30)
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
    
    if(Chaoche_start_time>0&&Chaoche_start_time<20)
      Chaoche_start_time++;
    else if(Chaoche_start_time>=20)    {
      Chaoche_start_time=0;
    }
    if(sum_time>0&&sum_time<100)
    {
      ChaoChe_temp=1;
      if(Car==1)
      {
        Servomiddle=8770;
      }
      else if(Car==2)
        Servomiddle=8850;
    }
    else
    {
      ChaoChe_temp=0;
    }*/
}

/*!
*  @brief      DMA0中断服务函数
*  @since      v5.0
*/
void DMA0_IRQHandler()
{
  camera_dma();
}

