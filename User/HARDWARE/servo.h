#ifndef __SERVO_H
#define __SERVO_H
#include "robocon.h"
void Servo_Init(uint16_t arr, uint16_t psc);

extern TIM_HandleTypeDef TIM4_Handler;
extern TIM_OC_InitTypeDef TIM4_CH1Handler;
extern TIM_OC_InitTypeDef TIM4_CH2Handler;

#define Servo1_CLOSE TIM4->CCR1=2420
#define Servo1_OPEN TIM4->CCR1=1420




//#define Servo2_PEAK_POS TIM4->CCR2=2420
//#define Servo2_DOWN_POS TIM4->CCR2=1700

//#define Servo3_CLOSE TIM4->CCR3=1080
//#define Servo3_OPEN TIM4->CCR3=600


#endif
