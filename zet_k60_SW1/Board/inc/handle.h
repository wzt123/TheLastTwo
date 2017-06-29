#ifndef __HANDLE_H__
#define __HANDLE_H__

#include "include.h"
#include "function.h"

#define  ROW_MAX    60
#define  COLUMN_MAX 80
#define line_width 2
//extern int CAMERA_H=60,CAMERA_W=80;
extern uint8 img[CAMERA_H][CAMERA_W];     //60*80  
extern int8 Road_Left[ROW_MAX];
extern int8 Road_Right[ROW_MAX];
extern int8 Road_Center[ROW_MAX];
extern int8 Road_Width[ROW_MAX];
extern uint8 Left_Flag[ROW_MAX];
extern uint8 Right_Flag[ROW_MAX];
extern uint16 Road_area;

extern uint8 Cross_Flag;
extern uint8 Change_Flag;
extern float Midd_Slope;
//              
extern uint8 Left_Cnt;
extern uint8 Right_Cnt;
extern uint8 Right_cnt;
extern uint8 Left_cnt;
extern uint8 Black_Cnt;
extern uint8 White_Cnt;
extern uint8 White_cnt;
extern uint8 All_Black;
extern uint8 All_White;
extern uint8 CutPos;
extern uint8 Road_Change;
extern uint8 Stop_Flag;
extern uint8 Road_type;
//extern uint8 l;
extern uint8 Lastline;
extern uint8 Car;
extern uint8 race[4];
extern uint8 buff[4];
extern uint8 Overtake_Flag;
extern uint8 Overtake2;
extern uint8 C;
extern uint16 Servomiddle;
extern uint8 Turn_Left;
extern uint8 Overtake;
extern uint8 Out_Left;
extern uint8 Out_Right;
extern uint8 Cross3_Cnt;
extern uint8 CrossRow;
//
extern uint8 GetMedianNum(uint8 *bArray,uint8 iFilterLen);
extern void Search_Line(void);
extern void Start_Line(void);
extern void Find_Middle(void);
extern void Edge_Filter(void);
extern void Servo_control(void);
extern void Overtake_judge();
extern void Cross_Search(void);
extern void Road_Type(void);
extern int16 error;
//////////////////////////////////////////
extern uint8 Bend_Right;
extern uint8 Bend_Lift;
extern float repair_slope;
extern uint8 Cross_Flag_Last;
extern uint8 Ring_First_Row;
extern int16 Servo_temp;
extern uint32 sum_time;
extern uint8 Ring_width;
extern int32 error1;
extern int32 error2;
extern uint8 cross_num;
extern uint8 white_Left_cnt;
extern uint8 white_Right_cnt;
extern uint8 Ring_width_1;
extern uint8 Ring_width_2;
extern uint8 Cross_Cnt;
extern uint8 stopLine_temp;
/////////////////////////////////////////
//extern int32 All_Black;
extern uint16 Servo_value;
extern int16 errorerror;
extern uint8 Kp;
extern uint8 Kd;
//extern float Servo_t;
extern int8   show_KJ;
#endif