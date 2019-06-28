
#include "power_ctrl.h"


void power_ctrl_on_all(void)
{
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5,GPIO_PIN_SET);  //开启四个24V输出

}

