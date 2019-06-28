
#include "buzzer.h"

TIM_HandleTypeDef TIM12_Handler;
TIM_OC_InitTypeDef TIM12_CH1Handler;


void ProcessBuzzer(void)
{

    int sound[]=
    {
        M7,750,Z0,

    };

    int length = sizeof(sound)/sizeof(sound[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {
        buzzer_on(sound[2*i], 200);
        vTaskDelay(4*sound[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
    buzzer_off();

}



void ActionDoneBuzzer(void)
{

    int sound[]=
    {
        M7,50,Z0,10,M7,50,Z0,10,M7,100,Z0,10,

    };

    int length = sizeof(sound)/sizeof(sound[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {
        buzzer_on(sound[2*i], 200);
        vTaskDelay(4*sound[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
    buzzer_off();

}


/**
* NAME: void BeginWarnBuzzer(void)
* FUNCTION : 开机提醒音 do mi xi
*/
void BeginWarnBuzzer(void)
{
    int sound[]=
    {
        M1,50,M3,50,M7,100,Z0,150,
        // H3,25,Z0,25, H3,25,Z0,25, H3,25,Z0,25, H3,25,Z0,100, H3,150,Z0,25,
    };

    int length = sizeof(sound)/sizeof(sound[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {
        buzzer_on(sound[2*i], 200);
        vTaskDelay(4*sound[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }

}

/**
* NAME: void IMUWarnBuzzer(void)
* FUNCTION : IMU校准提示音
*/
void IMUWarnBuzzer(void)
{
    int sound[]=
    {
        M7,50,Z0,150,
        H3,25,Z0,25, H3,25,Z0,25, H3,25,Z0,25, H3,25,Z0,100, H3,150,Z0,25,
    };

    int length = sizeof(sound)/sizeof(sound[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {
        buzzer_on(sound[2*i], 200);
        vTaskDelay(4*sound[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }

}

/**
* NAME: void happy_time(void)
* FUNCTION : happy
*/
void happy_time(void)
{
    int happy_birthday[]=
    {
        M5,50,M5,25,M5,25,
        M6,100,M5,100,H1,100,
        M7,100,M7,100,M5,50,M5,25,M5,25,
        M6,100,M5,100,H2,100,
        H1,100,H1,100,M5,50,M5,25,M5,25,
        H5,100,H3,100,H1,100,
        M7,100,M6,100,H4,50,H4,25,H4,25,
        H3,100,H1,100,H2,100,H1,100,H1,100
    };

    int length = sizeof(happy_birthday)/sizeof(happy_birthday[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {
        buzzer_on(happy_birthday[2*i], 200);
        vTaskDelay(5*happy_birthday[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}

/**
* NAME: void promising_young(void)
* FUNCTION : promising_young
*/
void promising_young(void)
{
    int promising_young[]=
    {
        M5,50,M6,50,H1,50,Z0,10,H2,50,   Z0,10,H2,50,Z0,1,H2,50, Z0,1, H1,50, Z0,1,H2,50,   Z0,15, H1,50, Z0,15,H3,50,  Z0,15,Z0,50,
        M7,40,Z0,10,M7,40,Z0,10,M7,40,Z0,10,M7,75,Z0,15,M7,50, Z0,25, M6,25, Z0,25, H1,25,Z0,25

    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}

void buzzer_init(uint16_t arr, uint16_t psc)
{

    TIM12_Handler.Instance=TIM12;            //定时器
    TIM12_Handler.Init.Prescaler=psc;       //定时器分频
    TIM12_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM12_Handler.Init.Period=arr;          //自动重装载值
    TIM12_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM12_Handler);       //初始化PWM

    TIM12_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM12_CH1Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM12_CH1Handler.OCPolarity=TIM_OCPOLARITY_LOW; //输出比较极性为低
    HAL_TIM_PWM_ConfigChannel(&TIM12_Handler,&TIM12_CH1Handler,TIM_CHANNEL_1);//配置TIM12通道1

    HAL_TIM_PWM_Start(&TIM12_Handler,TIM_CHANNEL_1);//开启PWM通道1

    buzzer_off();

}

void buzzer_on(uint16_t psc, uint16_t pwm)
{
    TIM12->PSC = psc;
    TIM12->CCR1=pwm;

}

void buzzer_off(void)
{
    TIM12->CCR1=0;
}

