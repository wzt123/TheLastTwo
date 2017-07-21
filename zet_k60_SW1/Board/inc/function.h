#ifndef __FUNCTION_H__
#define __FUNCTION_H__


extern void Init_All(void);
extern void Motor_Init(void);
extern void Motor_Out(void);
extern void uart3_handler(void);
extern void stop_Car2(void);
extern void stop();
extern void stop1(void);
extern void stop2(void);
extern void Distance_stop(void);
extern void xx_bluetooth();
extern void Switch_Init();
extern uint8 Get_Switch(void);
extern void Chaoche_FrontCar(void);
extern void NRF_SendData(int data);
extern void Ring_Overtake(void);
//extern uint16 speed_get;
extern uint16 var;//�����ࣻ
//extern uint16 speed_goal;
extern uint16 speed_goal_R;
extern uint16 speed_goal_L;
extern uint16 speed_goal;
extern uint8 Status;
extern uint8 Status_2;
extern int16 speed_PWM_R;
extern int16 speed_PWM_L;
extern uint8 stop_Flag;
extern uint8 stop_Place;
extern uint32 Distance;
extern uint8 Distance_temp;
extern uint8 Distance_temp_rember;
/////
extern int16 OutData[4];
extern uint16 speed_get_R;
extern uint16 speed_get_L;
extern uint16 send_data[3][8];
extern void stop_Car1();
extern uint8 Car_First_stop;
extern uint8 Car_Second_stop;
extern uint8 ChaoChe_stop_time;
extern uint8 ChaoChe_Cross_temp;
extern uint8 Distance_stop_temp;
extern uint8 Ring_OverTake;
////
#endif