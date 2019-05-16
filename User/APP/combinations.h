#ifndef COMBINATIONS_H
#define COMBINATIONS_H
#include "robocon.h"

//extern bool optoelec_switch[2];
#define keyToken HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_0)
#define keyStart HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_12)

#define keyRestart1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)
#define keyRestart2 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)
#define keyRestart3 HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5)

#define keyInf1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)
#define keyInf2 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)

#define RED_GROUNG 0
#define BLUE_GROUNG 1


#define       optoelec_switch[0]  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)
#define       optoelec_switch[1]  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4)

#define openmv_White 1
#define openmv_Yellow 2
#define openmv_Red 4

//#define openmv_Yellow 1
//#define openmv_Red 2


void StartJump(float start_time_s);
void TrajectoryJump(float t, float launchTime, float stanceHeight, float downAMP) ;
void ExecuteJump();
void StepOver(void);

void CrossTheLine(void);
void CrossTheLine_one_leg(int LegId);
void CrossTheLine_all(void);
void StepOver_one_leg(int LegId);

void StartPosToMiddlePos (void);
void MiddlePosToEndPos (void);

void OpenMvInspect(int color);
void Climbing_Test(void);
#endif
