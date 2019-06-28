
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
uint64_t times=0;		//ms使用tim3定时 产生稳定的脉冲周期
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

TIM_HandleTypeDef TIM2_Handler;
TIM_HandleTypeDef TIM3_Handler;      //定时器句柄

//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!(定时器3挂在APB1上，时钟为HCLK/2)           //
void TIM3_Init(uint16_t arr,uint16_t psc)
{
    TIM3_Handler.Instance=TIM3;                          //通用定时器3
    TIM3_Handler.Init.Prescaler=psc;                     //分频系数
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM3_Handler.Init.Period=arr;                        //自动装载值
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIM3_Handler);

    HAL_TIM_Base_Start_IT(&TIM3_Handler); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE
}


//arr：自动重装值(TIM2,TIM2是32位的!!)
//psc：时钟预分频数
void TIM2_CH3_Cap_Init(uint32_t arr,uint16_t psc)
{
    TIM_IC_InitTypeDef TIM2_CH3Config;

    TIM2_Handler.Instance=TIM2;                          //通用定时器5
    TIM2_Handler.Init.Prescaler=psc;                     //分频系数
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM2_Handler.Init.Period=arr;                        //自动装载值
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频银子
    HAL_TIM_IC_Init(&TIM2_Handler);//初始化输入捕获时基参数

    TIM2_CH3Config.ICPolarity=TIM_ICPOLARITY_RISING;    //上升沿捕获
    TIM2_CH3Config.ICSelection=TIM_ICSELECTION_DIRECTTI;//映射到TI1上
    TIM2_CH3Config.ICPrescaler=TIM_ICPSC_DIV1;          //配置输入分频，不分频
    TIM2_CH3Config.ICFilter=0;                          //配置输入滤波器，不滤波
    HAL_TIM_IC_ConfigChannel(&TIM2_Handler,&TIM2_CH3Config,TIM_CHANNEL_3);//配置TIM2通道3

    HAL_TIM_IC_Start_IT(&TIM2_Handler,TIM_CHANNEL_3);   //开启TIM2的捕获通道3，并且开启捕获中断
    __HAL_TIM_ENABLE_IT(&TIM2_Handler,TIM_IT_UPDATE);   //使能更新中断
}

//此函数会被HAL_TIM_IC_Init()调用
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM2_CLK_ENABLE();            //使能TIM2时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟

//    /**TIM2 GPIO Configuration
//    PA2     ------> TIM2_CH3
//    PA3     ------> TIM2_CH4
    GPIO_Initure.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    GPIO_Initure.Alternate=GPIO_AF1_TIM2;   //PA0复用为TIM2通道1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM2_IRQn,5,0);    //设置中断优先级，抢占优先级2，子优先级0
    HAL_NVIC_EnableIRQ(TIM2_IRQn);          //开启ITM5中断通道
}


//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM3时钟
        HAL_NVIC_SetPriority(TIM3_IRQn,1,3);    //设置中断优先级，抢占优先级1，子优先级3
        HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM3中断
    }
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM2_Handler);
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM3_Handler);
}


//回调函数，定时器中断服务函数调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&TIM3_Handler))
    {

    }
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}

//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
uint8_t  TIM2CH3_CAPTURE_STA=0;	//输入捕获状态
uint32_t	TIM2CH3_CAPTURE_VAL;	//输入捕获值(TIM2/TIM2是32位)

u32 temp_cap_tim2=0;
u8  ppm_rx_sta=0,ppm_rx_num=0;	//输入捕获状态
u16 ppm_rx[10];//ppm_rx[0]   1   接收到ppm数据


//定时器输入捕获中断处理回调函数，该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获中断发生时执行
{
    if((TIM2CH3_CAPTURE_STA&0X80)==0)//还未成功捕获
    {
        if(TIM2CH3_CAPTURE_STA&0X40)		//捕获到一个下降沿
        {
            TIM2CH3_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
            TIM2CH3_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&TIM2_Handler,TIM_CHANNEL_3);//获取当前的捕获值.
            TIM_RESET_CAPTUREPOLARITY(&TIM2_Handler,TIM_CHANNEL_3);   //一定要先清除原来的设置！！
            TIM_SET_CAPTUREPOLARITY(&TIM2_Handler,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING);//配置TIM2通道1上升沿捕获
        } else  								//还未开始,第一次捕获上升沿
        {
            TIM2CH3_CAPTURE_STA=0;			//清空
            TIM2CH3_CAPTURE_VAL=0;
            TIM2CH3_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
            __HAL_TIM_DISABLE(&TIM2_Handler);        //关闭定时器5
            __HAL_TIM_SET_COUNTER(&TIM2_Handler,0);
            TIM_RESET_CAPTUREPOLARITY(&TIM2_Handler,TIM_CHANNEL_3);   //一定要先清除原来的设置！！
            TIM_SET_CAPTUREPOLARITY(&TIM2_Handler,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);//定时器5通道1设置为下降沿捕获
            __HAL_TIM_ENABLE(&TIM2_Handler);//使能定时器5
        }
    }

    if(TIM2CH3_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
    {
        if(ppm_rx_sta==1) {
            ppm_rx[ppm_rx_num+1]=TIM2CH3_CAPTURE_VAL;	//printf("TIM2CH3_CAPTURE_VAL:%d\r\n",TIM2CH3_CAPTURE_VAL);
            ppm_rx_num++;
        }

        if(4>TIM2CH3_CAPTURE_STA&0X3F>0||TIM2CH3_CAPTURE_VAL>3000) ppm_rx_sta++;//低电平时间大于3000us为起始帧

        if(ppm_rx_sta==2) {
            ppm_rx_sta=0;    //printf("receive\r\n");//ppm_rx_sta   1 表示接收到同步帧/ 2接收到到下一起始帧 ppm数据接收完毕
            ppm_rx[0]=1;
            ppm_rx_num=0;
        }

        TIM2CH3_CAPTURE_STA=0;          //开启下一次捕获

    }

}

//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if(htim->Instance==TIM12)
    {
        __HAL_RCC_TIM12_CLK_ENABLE();			//使能定时器12
        __HAL_RCC_GPIOH_CLK_ENABLE();			//开启GPIOH时钟

        GPIO_InitStruct.Pin=GPIO_PIN_6;           	//PH6
        GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
        GPIO_InitStruct.Pull=GPIO_PULLUP;          //上拉
        GPIO_InitStruct.Speed=GPIO_SPEED_HIGH;     //高速
        GPIO_InitStruct.Alternate= GPIO_AF9_TIM12;	//PH6复用为TIM12_CH1
        HAL_GPIO_Init(GPIOH,&GPIO_InitStruct);
    }

    else if(htim->Instance==TIM4)
    {
        /**TIM4 GPIO Configuration
        PD15     ------> TIM4_CH4
        PD14     ------> TIM4_CH3
        PD13     ------> TIM4_CH2
        PD12     ------> TIM4_CH1
        */
        __HAL_RCC_TIM4_CLK_ENABLE();			//使能定时器12
        __HAL_RCC_GPIOD_CLK_ENABLE();			//开启GPIOD时钟

        GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    }

    else	if(htim->Instance==TIM2)
    {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    }

}
///* TIM2 init function */
//void MX_TIM2_Init(void)
//{
//    TIM_MasterConfigTypeDef sMasterConfig;
//    TIM_OC_InitTypeDef sConfigOC;

//    htim2.Instance = TIM2;
//    htim2.Init.Prescaler = 0;
//    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim2.Init.Period = 0;
//    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = 0;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    HAL_TIM_MspPostInit(&htim2);

//}


///* TIM3 init function */
//void MX_TIM3_Init(void)
//{
//    TIM_MasterConfigTypeDef sMasterConfig;
//    TIM_OC_InitTypeDef sConfigOC;

//    htim3.Instance = TIM3;
//    htim3.Init.Prescaler = 179;
//    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim3.Init.Period = 0;
//    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = 0;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    HAL_TIM_MspPostInit(&htim3);

//}
///* TIM4 init function */
//void MX_TIM4_Init(void)
//{
//    TIM_MasterConfigTypeDef sMasterConfig;
//    TIM_OC_InitTypeDef sConfigOC;

//    htim4.Instance = TIM4;
//    htim4.Init.Prescaler = 500-1;
//    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim4.Init.Period = 90-1;
//    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = 0;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    HAL_TIM_MspPostInit(&htim4);

//}
///* TIM5 init function */
//void MX_TIM5_Init(void)
//{
//    TIM_MasterConfigTypeDef sMasterConfig;
//    TIM_OC_InitTypeDef sConfigOC;

//    htim5.Instance = TIM5;
//    htim5.Init.Prescaler = 89;
//    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim5.Init.Period = 2000;//2000 = 500hz, 20000 = 50hz
//    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = 1000;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sConfigOC.Pulse = 0;
//    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    HAL_TIM_MspPostInit(&htim5);

//}
///* TIM8 init function */
//void MX_TIM8_Init(void)
//{
//    TIM_MasterConfigTypeDef sMasterConfig;
//    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
//    TIM_OC_InitTypeDef sConfigOC;

//    htim8.Instance = TIM8;
//    htim8.Init.Prescaler = 0;
//    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim8.Init.Period = 0;
//    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//    htim8.Init.RepetitionCounter = 0;
//    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//    sBreakDeadTimeConfig.DeadTime = 0;
//    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = 0;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    HAL_TIM_MspPostInit(&htim8);

//}
///* TIM12 init function */
//void MX_TIM12_Init(void)
//{
//    TIM_OC_InitTypeDef sConfigOC;

//    htim12.Instance = TIM12;
//    htim12.Init.Prescaler = 89;
//    htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim12.Init.Period = 2499;
//    htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//    if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = 1000;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    HAL_TIM_MspPostInit(&htim12);

//}

//__weak void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
//{

//    if(tim_pwmHandle->Instance==TIM2)
//    {
//        /* USER CODE BEGIN TIM2_MspInit 0 */

//        /* USER CODE END TIM2_MspInit 0 */
//        /* Peripheral clock enable */
//        __HAL_RCC_TIM2_CLK_ENABLE();
//        /* USER CODE BEGIN TIM2_MspInit 1 */

//        /* USER CODE END TIM2_MspInit 1 */
//    }
//    else if(tim_pwmHandle->Instance==TIM3)
//    {
//        /* USER CODE BEGIN TIM3_MspInit 0 */

//        /* USER CODE END TIM3_MspInit 0 */
//        /* Peripheral clock enable */
//        __HAL_RCC_TIM3_CLK_ENABLE();
//        /* USER CODE BEGIN TIM3_MspInit 1 */

//        /* USER CODE END TIM3_MspInit 1 */
//    }
//    else if(tim_pwmHandle->Instance==TIM4)
//    {
//        /* USER CODE BEGIN TIM4_MspInit 0 */

//        /* USER CODE END TIM4_MspInit 0 */
//        /* Peripheral clock enable */
//        __HAL_RCC_TIM4_CLK_ENABLE();
//        /* USER CODE BEGIN TIM4_MspInit 1 */

//        /* USER CODE END TIM4_MspInit 1 */
//    }
//    else if(tim_pwmHandle->Instance==TIM5)
//    {
//        /* USER CODE BEGIN TIM5_MspInit 0 */

//        /* USER CODE END TIM5_MspInit 0 */
//        /* Peripheral clock enable */
//        __HAL_RCC_TIM5_CLK_ENABLE();
//        /* USER CODE BEGIN TIM5_MspInit 1 */

//        /* USER CODE END TIM5_MspInit 1 */
//    }
//    else if(tim_pwmHandle->Instance==TIM8)
//    {
//        /* USER CODE BEGIN TIM8_MspInit 0 */

//        /* USER CODE END TIM8_MspInit 0 */
//        /* Peripheral clock enable */
//        __HAL_RCC_TIM8_CLK_ENABLE();
//        /* USER CODE BEGIN TIM8_MspInit 1 */

//        /* USER CODE END TIM8_MspInit 1 */
//    }
//    else if(tim_pwmHandle->Instance==TIM12)
//    {
//        /* USER CODE BEGIN TIM12_MspInit 0 */

//        /* USER CODE END TIM12_MspInit 0 */
//        /* Peripheral clock enable */
//        __HAL_RCC_TIM12_CLK_ENABLE();
//        /* USER CODE BEGIN TIM12_MspInit 1 */

//        /* USER CODE END TIM12_MspInit 1 */
//    }
//}
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
//{

//    GPIO_InitTypeDef GPIO_InitStruct;
//    if(timHandle->Instance==TIM2)
//    {
//        /* USER CODE BEGIN TIM2_MspPostInit 0 */

//        /* USER CODE END TIM2_MspPostInit 0 */
//        /**TIM2 GPIO Configuration
//        PB3     ------> TIM2_CH2
//        PA15     ------> TIM2_CH1
//        PB10     ------> TIM2_CH3
//        PB11     ------> TIM2_CH4
//        */
//        GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
//        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//        GPIO_InitStruct.Pin = GPIO_PIN_15;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
//        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//        /* USER CODE BEGIN TIM2_MspPostInit 1 */

//        /* USER CODE END TIM2_MspPostInit 1 */
//    }
//    else if(timHandle->Instance==TIM3)
//    {
//        /* USER CODE BEGIN TIM3_MspPostInit 0 */

//        /* USER CODE END TIM3_MspPostInit 0 */

//        /**TIM3 GPIO Configuration
//        PB5     ------> TIM3_CH2
//        PB4     ------> TIM3_CH1
//        */
//        GPIO_InitStruct.Pin = IMU_HEAT_PWM_Pin|BEEP_Pin;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//        /* USER CODE BEGIN TIM3_MspPostInit 1 */

//        /* USER CODE END TIM3_MspPostInit 1 */
//    }
//    else if(timHandle->Instance==TIM4)
//    {
//        /* USER CODE BEGIN TIM4_MspPostInit 0 */

//        /* USER CODE END TIM4_MspPostInit 0 */

//        /**TIM4 GPIO Configuration
//        PB8     ------> TIM4_CH3
//        PB9     ------> TIM4_CH4
//        PD13     ------> TIM4_CH2
//        PD12     ------> TIM4_CH1
//        */
//        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//        GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_12;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

//        /* USER CODE BEGIN TIM4_MspPostInit 1 */

//        /* USER CODE END TIM4_MspPostInit 1 */
//    }
//    else if(timHandle->Instance==TIM5)
//    {
//        /* USER CODE BEGIN TIM5_MspPostInit 0 */

//        /* USER CODE END TIM5_MspPostInit 0 */

//        /**TIM5 GPIO Configuration
//        PI0     ------> TIM5_CH4
//        PH12     ------> TIM5_CH3
//        PH11     ------> TIM5_CH2
//        PH10     ------> TIM5_CH1
//        */
//        GPIO_InitStruct.Pin = GPIO_PIN_0;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
//        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

//        GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
//        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

//        /* USER CODE BEGIN TIM5_MspPostInit 1 */

//        /* USER CODE END TIM5_MspPostInit 1 */
//    }
//    else if(timHandle->Instance==TIM8)
//    {
//        /* USER CODE BEGIN TIM8_MspPostInit 0 */

//        /* USER CODE END TIM8_MspPostInit 0 */

//        /**TIM8 GPIO Configuration
//        PI7     ------> TIM8_CH3
//        PI6     ------> TIM8_CH2
//        PI5     ------> TIM8_CH1
//        PI2     ------> TIM8_CH4
//        */
//        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_2;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
//        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

//        /* USER CODE BEGIN TIM8_MspPostInit 1 */

//        /* USER CODE END TIM8_MspPostInit 1 */
//    }
//    else if(timHandle->Instance==TIM12)
//    {
//        /* USER CODE BEGIN TIM12_MspPostInit 0 */

//        /* USER CODE END TIM12_MspPostInit 0 */

//        /**TIM12 GPIO Configuration
//        PH6     ------> TIM12_CH1
//        PH9     ------> TIM12_CH2
//        */
//        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_9;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
//        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

//        /* USER CODE BEGIN TIM12_MspPostInit 1 */

//        /* USER CODE END TIM12_MspPostInit 1 */
//    }

//}

//void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
//{

//    if(tim_pwmHandle->Instance==TIM2)
//    {
//        /* USER CODE BEGIN TIM2_MspDeInit 0 */

//        /* USER CODE END TIM2_MspDeInit 0 */
//        /* Peripheral clock disable */
//        __HAL_RCC_TIM2_CLK_DISABLE();
//        /* USER CODE BEGIN TIM2_MspDeInit 1 */

//        /* USER CODE END TIM2_MspDeInit 1 */
//    }
//    else if(tim_pwmHandle->Instance==TIM3)
//    {
//        /* USER CODE BEGIN TIM3_MspDeInit 0 */

//        /* USER CODE END TIM3_MspDeInit 0 */
//        /* Peripheral clock disable */
//        __HAL_RCC_TIM3_CLK_DISABLE();
//        /* USER CODE BEGIN TIM3_MspDeInit 1 */

//        /* USER CODE END TIM3_MspDeInit 1 */
//    }
//    else if(tim_pwmHandle->Instance==TIM4)
//    {
//        /* USER CODE BEGIN TIM4_MspDeInit 0 */

//        /* USER CODE END TIM4_MspDeInit 0 */
//        /* Peripheral clock disable */
//        __HAL_RCC_TIM4_CLK_DISABLE();
//        /* USER CODE BEGIN TIM4_MspDeInit 1 */

//        /* USER CODE END TIM4_MspDeInit 1 */
//    }
//    else if(tim_pwmHandle->Instance==TIM5)
//    {
//        /* USER CODE BEGIN TIM5_MspDeInit 0 */

//        /* USER CODE END TIM5_MspDeInit 0 */
//        /* Peripheral clock disable */
//        __HAL_RCC_TIM5_CLK_DISABLE();
//        /* USER CODE BEGIN TIM5_MspDeInit 1 */

//        /* USER CODE END TIM5_MspDeInit 1 */
//    }
//    else if(tim_pwmHandle->Instance==TIM8)
//    {
//        /* USER CODE BEGIN TIM8_MspDeInit 0 */

//        /* USER CODE END TIM8_MspDeInit 0 */
//        /* Peripheral clock disable */
//        __HAL_RCC_TIM8_CLK_DISABLE();
//        /* USER CODE BEGIN TIM8_MspDeInit 1 */

//        /* USER CODE END TIM8_MspDeInit 1 */
//    }
//    else if(tim_pwmHandle->Instance==TIM12)
//    {
//        /* USER CODE BEGIN TIM12_MspDeInit 0 */

//        /* USER CODE END TIM12_MspDeInit 0 */
//        /* Peripheral clock disable */
//        __HAL_RCC_TIM12_CLK_DISABLE();
//        /* USER CODE BEGIN TIM12_MspDeInit 1 */

//        /* USER CODE END TIM12_MspDeInit 1 */
//    }
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
