
#include "servo.h"

TIM_HandleTypeDef TIM4_Handler;
TIM_OC_InitTypeDef TIM4_CH1Handler;
TIM_OC_InitTypeDef TIM4_CH2Handler;
TIM_OC_InitTypeDef TIM4_CH3Handler;

void Servo_Init(uint16_t arr, uint16_t psc)
{

    TIM4_Handler.Instance=TIM4;            //定时器
    TIM4_Handler.Init.Prescaler=psc;       //定时器分频
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM4_Handler.Init.Period=arr;          //自动重装载值
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM4_Handler);       //初始化PWM

    TIM4_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM4_CH1Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM4_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为低
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH1Handler,TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_1);//开启PWM通道1

    TIM4_CH2Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM4_CH2Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM4_CH2Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为低
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH2Handler,TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_2);//开启PWM通道2

    TIM4_CH3Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM4_CH3Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM4_CH3Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为低
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH3Handler,TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_3);//开启PWM通道3

}
