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

  PIT_Flag_Clear(PIT0);       //���жϱ�־λ
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
  pit_init_ms(PIT0, 50);                             //��ʼ��PIT0����ʱʱ��Ϊ�� 1000ms       
  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);       //����PIT0���жϷ�����Ϊ PIT_IRQHandler
  stopLine_init();
  Switch_Init();
  while(!nrf_init());
  set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);                //���� PORTE ���жϷ�����Ϊ PORTE_VECTORn
  chaoShenBo_init();
  Status=Get_Switch();
  enable_irq(PORTC_IRQn);
  enable_irq (PIT0_IRQn);                                //ʹ��PIT0�ж�
}


/*
*       ��������ʼ��
*
*/
void Motor_Init(void)
{
    ftm_pwm_init(FTM2,FTM_CH1,15000,0);//����B1FTM��ʼ��
    gpio_init(PTC3,GPO,0);//��������ʹ�ܳ�ʼ��
    gpio_init(PTC2,GPO,0);//��������ʹ�ܳ�ʼ��
    gpio_set(PTC3,1);
    gpio_set(PTC2,0);
    
    ftm_pwm_init(FTM2,FTM_CH0,15000,0);//����B2FTM��ʼ��
    gpio_init(PTB17,GPO,0);//��������ʹ�ܳ�ʼ��
    gpio_init(PTB16,GPO,0);//��������ʹ�ܳ�ʼ��
    gpio_set(PTB17,0);
    gpio_set(PTB16,1);
    
    ftm_pwm_init(FTM0,FTM_CH3,100,0);
    //DELAY_MS(10);
    
    ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle);
    
}
/*
*       ���PID���
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
      if(abs(error)<4)//ֱ��
      {
        if(Status==0)
        {
          speed_goal=3800;
        }
        else if(Status==1)
        {
          speed_goal=4100;
        }
        else if(Status==2)//ֱ������
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
      
      else//����ٶ�
      {
       if(Status==0)
        {
          speed_goal=3800;
        }
        else if(Status==1)
        {
          speed_goal=4100;
        }
        else if(Status==2)//�������
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
    gpio_set(PTC3,0);//��������ʹ��
    gpio_set(PTC2,1);//��������ʹ��
    gpio_set(PTB17,1);//��������ʹ��
    gpio_set(PTB16,0);//��������ʹ��
    speed_PWM_R=abs(speed_PWM_R);
    
  }
  if(speed_PWM_R>8800)
    speed_PWM_R=8800;
  
  if(speed_PWM_L<0)
  {
    gpio_set(PTC3,0);//��������ʹ��
    gpio_set(PTC2,1);//��������ʹ��
    gpio_set(PTB17,1);//��������ʹ��
    gpio_set(PTB16,0);//��������ʹ��
    speed_PWM_L = abs(speed_PWM_L);
  }
  if(speed_PWM_L>8800)
    speed_PWM_L=8800;
   
  ftm_pwm_duty(FTM2,FTM_CH0,speed_PWM_L);//B2����
  ftm_pwm_duty(FTM2,FTM_CH1,speed_PWM_R);//B1�ҵ��
  
}
/*
*��ɲ
*/
void stop1(void)
{
  //gpio_set(PTC3,0);//��������ʹ��
  //gpio_set(PTC2,1);//��������ʹ��
  //gpio_set(PTB17,1);//��������ʹ��
  //gpio_set(PTB16,0);//��������ʹ��
  ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,0);//B1
  stop_time++;
  //DELAY_MS(10);
  gpio_set(PTC3,1);//��������ʹ��
  gpio_set(PTC2,0);//��������ʹ��
  gpio_set(PTB17,0);//��������ʹ��
  gpio_set(PTB16,1);//��������ʹ��
}
  
/*
*�㷴ɲ
*/
void stop2(void)
{
   gpio_set(PTC3,0);//��������ʹ��
  gpio_set(PTC2,1);//��������ʹ��
  gpio_set(PTB17,1);//��������ʹ��
  gpio_set(PTB16,0);//��������ʹ��
  ftm_pwm_duty(FTM2,FTM_CH0,8800);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,8800);//B1
  stop_time++;
}
/*
*2��ͣ��
*/
void stop_Car2(void)
{ 
  gpio_set(PTC3,0);//��������ʹ��
  gpio_set(PTC2,1);//��������ʹ��
  gpio_set(PTB17,1);//��������ʹ��
  gpio_set(PTB16,0);//��������ʹ��
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
*1��ͣ��
*/
void stop_Car1()
{
  if(speed_get_L<8&&speed_get_R<8)
  {
    ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,0);//B1  
    //stop_Flag  = 1;
    DELAY_MS(1000);//��ʱ�����ã����Ժ���NRF֪ͨ�󳵽ӽ��ٷ���
    
    gpio_set(PTC3,1);
    gpio_set(PTC2,0);
    gpio_set(PTB17,0);
    gpio_set(PTB16,1);
    ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle);
    ftm_pwm_duty(FTM2,FTM_CH0,6800);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,6800);//B1
    DELAY_MS(800);
    
    gpio_set(PTC3,0);//��������ʹ��
    gpio_set(PTC2,1);//��������ʹ��
    gpio_set(PTB17,1);//��������ʹ��
    gpio_set(PTB16,0);//��������ʹ��
    ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,0);//B1
    Car_First_stop=2;
    stop_Flag  = 1;
    return;
  }
  
  stop();
}

/*
�󳵼�⵽С��һ�������ͣ��
*/
void Distance_stop(void)
{
  if(speed_get_L<8&&speed_get_R<8)
  {
    gpio_set(PTC3,0);//��������ʹ��
    gpio_set(PTC2,1);//��������ʹ��
    gpio_set(PTB17,1);//��������ʹ��
    gpio_set(PTB16,0);//��������ʹ��
    ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,0);//B1
    Distance_stop_temp=1;//��ֹ���ͣ��
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
      speed_goal_R=0;//�����ٶ�
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
    gpio_set(PTC3,0);//��������ʹ��
    gpio_set(PTC2,1);//��������ʹ��
    gpio_set(PTB17,1);//��������ʹ��
    gpio_set(PTB16,0);//��������ʹ��
    speed_PWM_R=abs(speed_PWM_R);
    
  }
  if(speed_PWM_R>8800)
    speed_PWM_R=8800;
  
  if(speed_PWM_L<0)
  {
    gpio_set(PTC3,0);//��������ʹ��
    gpio_set(PTC2,1);//��������ʹ��
    gpio_set(PTB17,1);//��������ʹ��
    gpio_set(PTB16,0);//��������ʹ��
    speed_PWM_L = abs(speed_PWM_L);
  }
  if(speed_PWM_L>8800)
    speed_PWM_L=8800;
  
  ftm_pwm_duty(FTM2,FTM_CH0,speed_PWM_L);//B2����
  ftm_pwm_duty(FTM2,FTM_CH1,speed_PWM_R);//B1�ҵ��
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
����
*/
void Chaoche_FrontCar(void)
{
  uint8 All_black_rember=All_Black;
  int16 error_rember=error;
  int16 errorerror_rember=errorerror;
  
  ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle-150);
  
  gpio_set(PTC3,0);//��������ʹ��
  gpio_set(PTC2,1);//��������ʹ��
  gpio_set(PTB17,1);//��������ʹ��
  gpio_set(PTB16,0);//��������ʹ��
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
  gpio_set(PTC3,0);//��������ʹ��
  gpio_set(PTC2,1);//��������ʹ��
  gpio_set(PTB17,1);//��������ʹ��
  gpio_set(PTB16,0);//��������ʹ��
  ftm_pwm_duty(FTM2,FTM_CH0,7000);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,7000);//B1
  DELAY_MS(180);
  
  ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,0);//B
  
  NRF_SendData(10002);//���ߺ���ʮ��·��
  
  DELAY_MS(400);
  
  Car=2;
  gpio_set(PTE25,0);//�رճ�����
  gpio_set(PTE24,0);
  Overtake++;
  uint8 time=0;
  do
  {
    
    error=0;
    errorerror=0;
    Cross_Flag=0;
    gpio_set(PTC3,0);//��������ʹ��
    gpio_set(PTC2,1);//��������ʹ��
    gpio_set(PTB17,1);//��������ʹ��
    gpio_set(PTB16,0);//��������ʹ��
    ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle-100);
    ftm_pwm_duty(FTM2,FTM_CH0,7000);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,7000);//B1
    
    camera_get_img();                                   //����ͷ��ȡͼ��
    img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //��ֵ��ͼ��
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
    gpio_set(PTC3,0);//��������ʹ��
    gpio_set(PTC2,1);//��������ʹ��
    gpio_set(PTB17,1);//��������ʹ��
    gpio_set(PTB16,0);//��������ʹ��
    ftm_pwm_duty(FTM0, FTM_CH3, Servomiddle-150);
    ftm_pwm_duty(FTM2,FTM_CH0,7000);//B2
    ftm_pwm_duty(FTM2,FTM_CH1,7000);//B1
    
    camera_get_img();                                   //����ͷ��ȡͼ��
    img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //��ֵ��ͼ��
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
  camera_get_img();                                   //����ͷ��ȡͼ��
  img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //��ֵ��ͼ��
  Search_Line();
  Find_Middle();
  Servo_control();
  ftm_pwm_duty(FTM2,FTM_CH0,9500);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,9500);//B1
  NRF_SendData(10001);//����2�������ɹ�
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
  while(nrf_tx_state() == NRF_TXING);//�ȴ��������
  
}




//���뿪�س�ʼ��

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
//��ȡ���뿪��ֵ
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
* ������ʼ��
*
*/
void xx_bluetooth()
{
  uart_init(UART5,9600);     //��ʼ������(UART3 �ǹ���������Ϊprintf��������˿ڣ����Ѿ����г�ʼ��)
  //uart_putstr   (UART5 ,"\n\n\n�����жϲ��ԣ�");           //�����ַ���
  set_vector_handler(UART5_RX_TX_VECTORn,uart5_handler);   // �����жϷ��������ж���������
  uart_rx_irq_en (UART5);                                 //�����ڽ����ж�
}

/*
* ���������жϷ�����
*/
void uart5_handler(void)
{
  char ch;
  
  if(uart_query    (UART5) == 1)   //�������ݼĴ�����
  {
    //�û���Ҫ�����������
    uart_getchar   (UART5, &ch);                    //���޵ȴ�����1���ֽ�
    uart_putchar   (UART5 , ch);                    //�����ַ���
    uart_putstr   (UART5 ,"\n\n\n�����жϲ��ԣ�");
  }
}


/*
* NRF�жϺ���
*/
void PORTC_IRQHandler()
{
  uint8  n;    //���ź�
  uint32 flag;
  
  flag = PORTC_ISFR;
  PORTC_ISFR  = ~0;                                   //���жϱ�־λ
  
  n = 0;
  if(flag & (1 << n))                                 //PTC0�����ж�
  {
    nrf_handler();
  }
  
}


/*
*��������PIT��ʼ��
*/
void ChaoShenBo_PitInit(PITn_e pitn)
{
  SIM_SCGC6   |= SIM_SCGC6_PIT_MASK;//module clock  
  PIT_MCR     &= ~PIT_MCR_MDIS_MASK;//pit module enable 
  PIT_LDVAL(pitn)   = 0xFFFFFFFF;          // 
  PIT_Flag_Clear(pitn);       //���жϱ�־λ
  //   PIT_TCTRL0  |= PIT_TCTRL_TIE_MASK;//Enable interrupt
  //   PIT_TCTRL0  |= PIT_TCTRL_TEN_MASK;//enable the timer,run
}
/*
* ��������ʼ��
*
*/
void chaoShenBo_init(void)
{
  gpio_init(PTE27,GPI,1);//ʹ��GPIOA���жϣ���ʼ��IO��Ϊ���룬����
  port_init(PTE27, IRQ_EITHER | PF | ALT1 | PULLUP);
  if(Car==1)
  {
    gpio_init(PTE25,GPO,1);//ǰ������
    gpio_init(PTE24,GPO,1);//ǰ������
    
  }
  else
  {
    gpio_init(PTE25,GPO,0);
    gpio_init(PTE24,GPO,0);
  }
  NVIC_SetPriority(PORTE_IRQn,0);//���������жϣ����ȼ���������Ϊ��ߵ�
  set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);                //���� PORTE ���жϷ�����Ϊ PORTE_VECTORn
  
  enable_irq(PORTE_IRQn);
  ChaoShenBo_PitInit(PIT2);  
  NVIC_SetPriority(PIT2_IRQn,3);//����ж�����ν,
}

void stopLine_init(void)
{
  gpio_init(PTE10,GPI,1);//ʹ��GPIOE���жϣ���ʼ��IO��Ϊ���룬����
  //port_init(PTE10, IRQ_RISING | PF | ALT1 | PULLUP);
  gpio_init(PTE9,GPI,1);
  //port_init(PTE9, IRQ_RISING | PF | ALT1 | PULLUP);
}