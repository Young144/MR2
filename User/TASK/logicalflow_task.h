#ifndef LOGICALFLOW_TASK_H
#define LOGICALFLOW_TASK_H
#include "robocon.h"


#define keyRestart1 HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_6)
#define keyRestart2 HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_0)
#define keyRestartclimb HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_11)

#define keyRestart3 HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5)

#define keyStart HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_12)


#define keyInf1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)
#define keyInf2 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)

#define RED_GROUNG 0
#define BLUE_GROUNG 1

extern bool GROUND_SELECT;  //∫Ï¿∂≥°—°‘Ò
extern bool restartflag;

//#define openmv_White 1
//#define openmv_Yellow 2
//#define openmv_Red 4

#define openmv_Yellow 1
#define openmv_Red 2


#endif
