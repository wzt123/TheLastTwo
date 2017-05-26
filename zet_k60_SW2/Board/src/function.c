#include "common.h"
#include "include.h"
#include "function.h"

#include "handle.h"

uint16 speed_goal_R;
uint16 speed_goal_L;
//uint16 speed_get;
uint16 speed_get_R=0;
uint16 speed_get_L=0;
uint16 speed_last=0;
uint16 speed_nexterr=0;
uint16 speed_lasterr=0;
uint16 speed_PWM=0;
uint16 speed_PWM_R = 0;
uint16 speed_PWM_L = 0;
int16  speed_err_R;
int16  speed_err_L;
int16 speed_increment_R;
int16 speed_increment_L;
uint8 Status=0;
uint16 var;
uint8 stop_Flag = 0;
uint8 stop_Place = 0;
uint32 Distance = 2500;
uint16 send_data[3][8] = { { 0 }, { 0 }, { 0 } };
   

void xx_bluetooth();
void uart3_handler(void);
extern uint8 imgbuff[CAMERA_SIZE];
void PORTC_IRQHandler();
void xx_bluetooth();
void uart5_handler(void);
void chaoShenBo_init(void);
void stopLine_init(void);
void PIT0_IRQHandler(void)
{

    //int16 val;
    //speed_get = ftm_quad_get(FTM1);          //��ȡFTM �������� ��������(������ʾ������)
  if(stop_Flag !=1)
  {  
    //Motor_Out();
  }
  if(Cross_Flag==3&&stop_Flag !=1&&stop_Place==1)
  {
    //stop_Car();
  }
    ftm_quad_clean(FTM1);
   // Search_Line();
    //Find_Middle();
    //Servo_control();
    //zf_oled( val);
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
}
void Init_All(void)
{
  Car=2;
  Stop_Flag=0;
  Motor_Init();
  OLED_Init();
  ov7725_eagle_init(imgbuff);
  xx_bluetooth();
  ftm_quad_init(FTM1);
  lptmr_pulse_init(LPT0_ALT2,0xFFFF,LPT_Rising);
  pit_init_ms(PIT0, 50);                             //��ʼ��PIT0����ʱʱ��Ϊ�� 1000ms       
  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);       //����PIT0���жϷ�����Ϊ PIT_IRQHandler
  stopLine_init();
  //while(!nrf_init());
  set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);                //���� PORTE ���жϷ�����Ϊ PORTE_VECTORn
  chaoShenBo_init();
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
    
    ftm_pwm_duty(FTM0, FTM_CH3, 8508);
    
}

  
/*
*       ���PID���
*
*/
void Motor_Out(void)
{
  speed_PWM=6900;
  uint8 speed_Ki=30;
  float speed_Kd=0.0;
  float speed_Kp=0.0;
       gpio_set(PTC3,1);
       gpio_set(PTC2,0);
       gpio_set(PTB17,0);
       gpio_set(PTB16,1);
    
       if(All_Black>=40)
        {speed_PWM=0;}
       else
       {
         if(speed_get_R<100||speed_get_L<100)
         {
           speed_PWM_R = 7000;
           speed_PWM_L = 7000;
         }
         else
         {           
           speed_goal_R=5200;
           speed_goal_L=5200;
           speed_err_R=speed_goal_R-speed_get_R*10;
           speed_err_L = speed_goal_L-speed_get_L*10;
           speed_increment_R=speed_Ki*speed_err_R/10;
           speed_increment_L=speed_Ki*speed_err_L/10;
           speed_PWM_R=6000+speed_increment_R;
           speed_PWM_L=6000+speed_increment_L;
         }
       }
      
       //speed_PWM_R = speed_PWM_R - error*abs(error)/8;
       //speed_PWM_L = speed_PWM_L + error*abs(error)/8;
       
       
      if(speed_PWM_R<0)
        speed_PWM_R=0;
        if(speed_PWM_R>8900)
          speed_PWM_R=8900;
        
      if(speed_PWM_L<0)
        speed_PWM_L=0;
      if(speed_PWM_L>8900)
         speed_PWM_L=8900;
        
      /*if(speed_get_R<10||speed_get_L<10)
         {
           speed_PWM_R = 0;
           speed_PWM_L = 0;
         }*/
   ftm_pwm_duty(FTM2,FTM_CH0,speed_PWM_L);//B2����
   ftm_pwm_duty(FTM2,FTM_CH1,speed_PWM_R);//B1�ҵ��
   
  
    //ftm_pwm_duty(FTM2,FTM_CH0,speed_PWM);//B2����
   //ftm_pwm_duty(FTM2,FTM_CH1,speed_PWM);//B1�ҵ��
   
}

/*
*ͣ��
*/
void stop_Car(void)
{
  gpio_set(PTC3,0);//��������ʹ��
  gpio_set(PTC2,1);//��������ʹ��
  gpio_set(PTB17,1);//��������ʹ��
  gpio_set(PTB16,0);//��������ʹ��
  ftm_pwm_duty(FTM2,FTM_CH0,speed_PWM);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,speed_PWM);//B1
  DELAY_MS(250);
  ftm_pwm_duty(FTM2,FTM_CH0,0);//B2
  ftm_pwm_duty(FTM2,FTM_CH1,0);//B1
  stop_Flag  = 1;
  //disable_irq(PIT0_IRQn);
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
    if(flag & (1 << n))                                 //PTE27�����ж�
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