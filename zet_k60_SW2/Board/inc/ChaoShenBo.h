#ifndef __CHAOSHENBO_H__
#define __CHAOSHENBO_H__

#include "common.h"
#include "include.h"

  #define  EXTI_Rising                  ((uint8)0x09)
  #define  EXTI_Falling                 ((uint8)0x0A) 
  #define  EXTI_Both                    ((uint8)0x0B)
extern uint32 chaoshengboTime;//��ȡ���ĳ�����ʱ��,��λ΢��
extern uint32 ABDistance;//�����ķ��ͽ���ģ��ľ���,��λ����


uint32 setdelay(unsigned long t);
uint16 checkdelay (unsigned long t);
uint32 GetSysMsTime(void);
uint32 GetSys100UsTime(void);
uint32 GetSys10UsTime(void);
extern void EXTI_SET(PTXn_e ptxn,uint8 IRQC);
extern void PORTE_IRQHandler(void);



#endif

