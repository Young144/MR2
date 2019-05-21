
#include "robocon.h"

/* TASK ------------------------------------------------------------------*/

#define START_TASK_PRIO		0		
#define START_STK_SIZE 		128  			
TaskHandle_t StartTask_Handler;		
void start_task(void *pvParameters);		

#define MotorControl_TASK_PRIO		4		
#define MotorControl_STK_SIZE 		256  
TaskHandle_t MotorControlTask_Handler;		
void MotorControl_task(void *pvParameters);	

#define PostureControl_TASK_PRIO		5		
#define PostureControl_STK_SIZE 		256 	 	
TaskHandle_t PostureControlTask_Handler;		
void PostureControl_task(void *pvParameters);		

#define Navi_TASK_PRIO		5		
#define Navi_STK_SIZE 		256 	 	
TaskHandle_t NaviTask_Handler;		
void Navi_task(void *pvParameters);		

#define Detect_TASK_PRIO		3		
#define Detect_STK_SIZE 		128 	 	
TaskHandle_t DetectTask_Handler;		
void Detect_task(void *pvParameters);		

#define Debug_TASK_PRIO		6		
#define Debug_STK_SIZE 		256  
TaskHandle_t DebugTask_Handler;		
void Debug_task(void *pvParameters);		

#define Rc_TASK_PRIO		6		
#define Rc_STK_SIZE 		256  	
TaskHandle_t RcTask_Handler;		
void Rc_task(void *pvParameters);		

#define VcanGC_TASK_PRIO		6		
#define VcanGC_STK_SIZE 		256 	 	
TaskHandle_t VcanGCTask_Handler;		
void VcanGC_task(void *pvParameters);		

#define Test_TASK_PRIO		6		
#define Test_STK_SIZE 		256 	 	
TaskHandle_t TestTask_Handler;		
void Test_task(void *pvParameters);		

#define LogicalFlow_TASK_PRIO		6		
#define LogicalFlow_STK_SIZE 		256 	 	
TaskHandle_t LogicalFlowTask_Handler;		
void LogicalFlow_task(void *pvParameters);	


void SystemClock_Config(void);
static void MX_NVIC_Init(void);

int main(void)
{
    HAL_Init();						//Hal库初始化
    SystemClock_Config();	//系统时钟初始化
    MX_GPIO_Init();				//GPIO初始化
    MX_DMA_Init();				//DMA初始化
    MX_CAN1_Init();				//CAN1接口初始化

    MX_CAN2_Init();				//CAN1接口初始化
    MX_SPI5_Init();				//spi5初始化

    MX_USART2_UART_Init();
    uart_receive_init(&USART2_HUART);//USART2DMA空闲中断
    MX_USART3_UART_Init();
    uart_receive_init(&USART3_HUART);//USART3DMA空闲中断
    MX_USART6_UART_Init();
    uart_receive_init(&IMU_HUART);//usart6DMA接收mti30数据
    MX_UART7_Init();			//uart7DMA接收
    uart_receive_init(&OPENMV_HUART);//USART7DMA空闲中断
    MX_UART8_Init();		//usart8DMA发送数据给山外上位机

    MX_NVIC_Init();				//中断优先级初始化
    //定时器时钟90M分频系数9000-1,定时器3的频率90M/9000=10K自动重装载10-1,定时器周期1ms
    TIM3_Init(10-1,9000-1);//定时器3初始化 步态震荡时钟

    my_can_filter_init_recv_all(&hcan1);//开启CAN滤波器
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);//开启CAN1
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);//开启CAN2

    pid_param_init();		//所有使用到的PID参数初始化
    buzzer_init(500-1, 90-1);//蜂鸣器初始化
    led_configuration();	//流水灯 红绿灯初始化
    //f=Tck/(psc+1)*(arr+1) 定时器时钟为90M  50Hz=180MHz/(90*20000)
    Servo_Init(20000-1,90-1);//舵机PWM通道初始化

    //TIM2_CH3_Cap_Init(0XFFFFFFFF,90-1); //PWM捕获 以1MHZ的频率计数

    Servo1_DOWN;//舵机1复位
    Servo2_DOWN_POS;//舵机2复位
    Servo3_CLOSE;

//    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5,GPIO_PIN_SET);  //开启四个24V输出

    printf("\r\n/*************SYSTEM INIT SUCCESS****************/ \r\n");

    //创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄
    vTaskStartScheduler();          //开启任务调度

}

//开始任务任务函数
void start_task(void *pvParameters)
{
    BeginWarnBuzzer();

    taskENTER_CRITICAL();           //进入临界区
    //创建MotorControl_task
    xTaskCreate((TaskFunction_t )MotorControl_task,
                (const char*    )"MotorControl_task",
                (uint16_t       )MotorControl_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )MotorControl_TASK_PRIO,
                (TaskHandle_t*  )&MotorControlTask_Handler);
//    //创建PostureControl_task
//    xTaskCreate((TaskFunction_t )PostureControl_task,
//                (const char*    )"PostureControl_task",
//                (uint16_t       )PostureControl_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )PostureControl_TASK_PRIO,
//                (TaskHandle_t*  )&PostureControlTask_Handler);
//    //创建NAVIGATION任务
//    xTaskCreate((TaskFunction_t )Navi_task,
//                (const char*    )"Navi_task",
//                (uint16_t       )Navi_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )Navi_TASK_PRIO,
//                (TaskHandle_t*  )&NaviTask_Handler);
    //创建Detect任务
//    xTaskCreate((TaskFunction_t )Detect_task,
//                (const char*    )"Detect_task",
//                (uint16_t       )Detect_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )Detect_TASK_PRIO,
//                (TaskHandle_t*  )&DetectTask_Handler);
    //创建Debug_task
    xTaskCreate((TaskFunction_t )Debug_task,
                (const char*    )"Debug_task",
                (uint16_t       )Debug_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )Debug_TASK_PRIO,
                (TaskHandle_t*  )&DebugTask_Handler);
//    //创建Rc_task
//    xTaskCreate((TaskFunction_t )Rc_task,
//                (const char*    )"Rc_task",
//                (uint16_t       )Rc_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )Rc_TASK_PRIO,
//                (TaskHandle_t*  )&RcTask_Handler);
    //创建VcanGC任务 VCAN ground control 山外上位机
    xTaskCreate((TaskFunction_t )VcanGC_task,
                (const char*    )"VcanGC_task",
                (uint16_t       )VcanGC_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )VcanGC_TASK_PRIO,
                (TaskHandle_t*  )&VcanGCTask_Handler);
    //创建Test任务
    xTaskCreate((TaskFunction_t )Test_task,
                (const char*    )"Test_task",
                (uint16_t       )Test_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )Test_TASK_PRIO,
                (TaskHandle_t*  )&TestTask_Handler);
//    //创建LogicalFlow任务 逻辑流控制
//    xTaskCreate((TaskFunction_t )LogicalFlow_task,
//                (const char*    )"LogicalFlow_task",
//                (uint16_t       )LogicalFlow_STK_SIZE,
//                (void*           )NULL,
//                (UBaseType_t    )LogicalFlow_TASK_PRIO,
//                (TaskHandle_t*  )&LogicalFlowTask_Handler);

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区

}


void Test_task(void *pvParameters)
{

    float kalam;
    for(;;) {

//        if(keyRestart3==0) {
//            vTaskDelay(200);
//            if(keyRestart3==0) {
//                IndicateLED_On;
//                state= STOP;
//                vTaskDelay(800);
//                vTaskDelete(LogicalFlowTask_Handler);
//                vTaskDelay(200);

//                taskENTER_CRITICAL();           //进入临界区
//                xTaskCreate((TaskFunction_t )LogicalFlow_task,
//                            (const char*    )"LogicalFlow_task",
//                            (uint16_t       )LogicalFlow_STK_SIZE,
//                            (void*           )NULL,
//                            (UBaseType_t    )LogicalFlow_TASK_PRIO,
//                            (TaskHandle_t*  )&LogicalFlowTask_Handler);
//                taskEXIT_CRITICAL();            //退出临界区

//                IndicateLED_Off;

//            }
//        }

//test_speed+=200;
temp_pid.ref_agle[0]=0;
IsMotoReadyOrNot= IsReady;
        vTaskDelay(1500);
				
temp_pid.ref_agle[0]=100000;
IsMotoReadyOrNot= IsReady;
				vTaskDelay(1500);
//								if(temp_pid.ref_agle[0]>=8000)
//				{
//					temp_pid.ref_agle[0]=0;
//				}
								
//				if(test_speed>=8000)
//				{
//					test_speed=0;
//				}

//StartPosToMiddlePos();

//printf("Pitch %f Roll %f Yaw %f stage %d count %d\r\n",imuinfo.ActVal[1],imuinfo.ActVal[2],imuinfo.ActVal[0],stage,_count_navi);
// printf("_Pitch_rev %f  pitch_offset %f qian%d hou %d\r\n",_Pitch_rev,pitch_offset,HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_2),HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_7));

//printf("Roll %f Pitch %f Yaw %f dev_len %f yaw_calibrated %f\r\n",imuinfo.ActVal[2],imuinfo.ActVal[1],imuinfo.ActVal[0],step_len_dev,yaw_calibrated );

//printf("Roll %f Pitch %f Yaw %f\r\n",imuinfo.ActVal[2],imuinfo.ActVal[1],imuinfo.ActVal[0]);

//printf("Yaw %f dev_high %f\r\n",imuinfo.ActVal[0],step_high_dev );

//
//        printf("Yaw %f step_len_dev %f 偏心度%f  白线角%f  色块%f \r\n",imuinfo.ActVal[0],step_len_dev,openmvinfo.ActVal[0],openmvinfo.ActVal[1],openmvinfo.ActVal[2]);

//				printf(" CCR1 %d  CCR2 %d  CCR3 %d\r\n",(int)TIM4->CCR1,(int)TIM4->CCR2,(int)TIM4->CCR3);
//        printf(" tockenA %d startB %d restart1U %d restart2V %d restart3W %d	restartclimbc %d inf1S %d inf2T %d\r\n",HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_0),HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_12),HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2),HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3),HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5),HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_11),HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0),HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1));

//        printf("步高 %2.1f  ",state_detached_params[state].detached_params_0.stance_height);
//        printf("步长 %2.1f  ",state_detached_params[state].detached_params_0.step_length);
//        printf("抬腿高 %2.1f  ",state_detached_params[state].detached_params_0.up_amp);
//        printf("压腿高 %2.1f  ",state_detached_params[state].detached_params_0.down_amp);
//        printf("飞行占比 %2.2f  ",state_detached_params[state].detached_params_0.flight_percent);
//        printf("频率 %2.1f  ",state_detached_params[state].detached_params_0.freq);

//        printf("\r\n");



//printf("1 %d 2 %d 3 %d \r\n",USART6RxBuf[0],USART6RxBuf[1],USART6RxBuf[2]);
//kalam=KalmanFilter(imuinfo.ActVal[0],KALMAN_Q,KALMAN_R);
//printf("KEY_VALUE %d Yaw %f  Yaw_Calibrated %f step_len_dev %f",ps2info.KEY_VALUE, imuinfo.ActVal[0],Yaw_Calibrated,step_len_dev);
//printf("KEY_VALUE=%d Yaw %f",ps2info.KEY_VALUE, imuinfo.ActVal[0]);


//		  printf("yaw_set=%f ",yaw_set);
//      printf("\r\n");
//			for(int i=0;i<8;i++)
//			printf("%x ", usart3_buf[i]);
//			printf("\r\n");

//			for(int i=0;i<28;i++)
//			printf("%x ",imu_buf[i]);
//			printf("\r\n");


//			for(int i=0;i<16;i++)
//			printf("%x ",openmv_buf[i]);
//			printf("\r\n");		//角度  //中心坐标 // 1白 2黄 4红





//      printf(" %f  %f  %f\r\n",KalmanFilter(imuinfo.ActVal[2],KALMAN_Q,KALMAN_R),KalmanFilter(imuinfo.ActVal[1],KALMAN_Q,KALMAN_R),KalmanFilter(imuinfo.ActVal[0],KALMAN_Q,KALMAN_R));
//      printf("Roll %f Pitch %f Yaw %f \r\n",imuinfo.ActVal[2],imuinfo.ActVal[1],imuinfo.ActVal[0]);
//      printf(" %x %x %x  %x %x %x %x %x %x\n",dbus_buf[0],dbus_buf[1],dbus_buf[2],dbus_buf[3],dbus_buf[4],dbus_buf[5],dbus_buf[6],dbus_buf[7],dbus_buf[8]);

//		mpu_get_data();
//		imu_ahrs_update();
//		imu_attitude_update();
//		HAL_Delay(5);
//		printf(" Roll:  \n");
//		HAL_UART_Transmit(&huart6, (uint8_t *)buf, (COUNTOF(buf)-1), 55);
//		HAL_Delay(5);

//		Servo_DOWN;
//		wave_form_data[1] =count;
//		wave_form_data[2] =count;
//    temp_pid.ref_agle[0]-=30.0f*ReductionAndAngleRatio;
//    temp_pid.ref_agle[1]+=30.0f*ReductionAndAngleRatio;

//		temp_pid.ref_agle[0]=count*TransData;
//		count+=10;
//		if(count>=50)
//		{count=0;}

//		printf(" ref_agle[0]  = %f   ref_agle[1]  =%f  \n",temp_pid.ref_agle[0],temp_pid.ref_agle[1]);
//		printf(" theat1  = %f   theat2  =%f  \n",theta1,theta2);

//		Servo_PEAK;

    }
}

/////--------转向------1-----///
//        _rotate_angle=20;		//转向
//        yaw_set=20;
//        state = ROTAT_LEFT;
//        while(imuinfo.ActVal[0]<=_rotate_angle)
//            osDelay(50);
//
//        state= STOP;
//        vTaskDelay(500);

//        state= TROT;		//继续走直到转弯检测

//        while(1)  //等待白线角度>=80
//        {
//            vTaskDelay(300);
//            if(openmvinfo.ActVal[1]>=80)
//            {
//                vTaskDelay(300);
//                if(openmvinfo.ActVal[1]>=80)
//                {
//                    vTaskDelay(300);
//                    if(openmvinfo.ActVal[1]>=80)
//                        break;
//                }
//            }
//        }

//				vTaskDelay(2000);
//        state=STOP;
//        state=REALSE;


//        _rotate_angle=0;		//转向
//        yaw_set=0;
//        state = ROTAT_RIGHT;

//        while(imuinfo.ActVal[0]>=_rotate_angle)
//            osDelay(50);
//        state= STOP;

//        vTaskDelay(300);
//        state= TROT;



void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}
static void MX_NVIC_Init(void)
{
    /* USART1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* USART2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* USART3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* USART6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

    /* USART7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);

    /* DMA1_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    /* DMA2_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);


}

void Error_Handler(void)
{
    while(1)
    {
    }
    /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
