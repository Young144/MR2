
#include "moto_ctrl.h"

void send_chassis_cur1_4(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void send_chassis_cur5_8(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);

bool IsMotoReadyOrNot= NotReady;
float ref_agle[8]= {0};
float temp_angle;
temp_data temp_pid= {0};     //pid中间数据

float test_speed=0;
u16 pid_spd_out_limit = 6720;
/*******************************************************************************************
	*@ 函数名称：void MotorControl_task(void *pvParameters)
	*@ 功能： 接受电机反馈数据并且进行PID计算 输出电流大小控制电机位置
	*@ 备注： FREERTOS任务函数
 *******************************************************************************************/
void MotorControl_task(void *pvParameters)
{
    for(;;) {
        moto_behaviour();
    }

}

void moto_behaviour(void)
{
    if(IsMotoReadyOrNot== IsReady) {
        for(int i=0; i<8; i++)
            ref_agle[i]=temp_pid.ref_agle[i];
        IsMotoReadyOrNot= NotReady;
    }

    for(int i=0; i<8; i++) {

        //pid_calc(&pid_pos[i],moto_chassis[i].total_angle/100,ref_agle[i]/100);  //位置环 角度控制
        pid_calc(&pid_pos[i],moto_chassis[i].total_angle*360/8191,ref_agle[i]*360/8191);  //位置环 角度控制

        if(pid_pos[i].pos_out>pid_spd_out_limit)pid_pos[i].pos_out=pid_spd_out_limit;	//最高速度限制
        else if(pid_pos[i].pos_out<-pid_spd_out_limit)pid_pos[i].pos_out=-pid_spd_out_limit; //最高速度限制

        //temp_pid.out[i] = pid_calc(&pid_spd[i],moto_chassis[i].speed_rpm,test_speed);  //速度环 测试
        //temp_pid.out[i] = pid_calc(&pid_spd[i],moto_chassis[i].speed_rpm,pid_pos[i].pos_out);  //速度环 速度控制
        moto_chassis[i].given_current = pid_calc(&pid_spd[i],moto_chassis[i].speed_rpm,pid_pos[i].pos_out);  //速度环 速度控制
    }

    send_chassis_cur5_8(moto_chassis[4].given_current,moto_chassis[5].given_current,moto_chassis[6].given_current,moto_chassis[7].given_current);		//传递5-8数据给can收发器
    send_chassis_cur1_4(moto_chassis[0].given_current,moto_chassis[1].given_current,moto_chassis[2].given_current,moto_chassis[3].given_current);		//传递1-4数据给can收发器

    osDelay(3);		//控制频率  给	1	7号和8号电机会失控

}

/**
* NAME: void pid_param_init(void)
* FUNCTION : pid参数填充初始化
*/
void pid_param_init(void)
{

    for (int i = 0; i < 8; i++)//  						20,0.01,0  37,0.008   8.0f, 0.000f   16
        PID_struct_init(&pid_pos[i], POSITION_PID, 100000.0f, 2000.0f, 8.0f, 0.0008f, 0.0f);  //位置环PID参数设置（pid结构体，PID类型，最大输出，比例限制，P , I , D ）

    for (int i = 0; i < 8; i++)  //									16384.0f对应20A					15.5f,0,0 // 16.0f, 0.001f   20.0f, 0.004297f   22.0f, 0.01399f
        PID_struct_init(&pid_spd[i], POSITION_PID, 16000.0f, 2000.0f, 15.5f, 0.000100, 0.0f);		//速度环PID（pid结构体，PID类型，最大输出，比例限制，P , I , D ）
    //20.0f, 0.004297f, 0.0f     22.0f, 0.001

    PID_struct_init(&pid_imu[0], POSITION_PID, 10000, 0, 1.5f, 0.01, 0);		//roll

    PID_struct_init(&pid_imu[1], POSITION_PID, 10000, 0, 1.5f, 0.01, 0);		//pitch

    PID_struct_init(&pid_imu[2], POSITION_PID, 10000, 0, 1.30f, 0, 0);		//yaw

    PID_struct_init(&pid_climbing, POSITION_PID, 10000, 10000, 3.5f, 0, 0);		//yaw 1.8

    PID_struct_init(&pid_test1, POSITION_PID, 10000, 0, 1.5f, 0, 0);		//yaw

    PID_struct_init(&pid_openmv_dev, POSITION_PID, 10000, 0, 1.5f, 0, 0);		//yaw
}

//发送底盘电机控制命令
void send_chassis_cur1_4(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    hcan1.pTxMsg->StdId   = 0x200;
    hcan1.pTxMsg->IDE     = CAN_ID_STD;
    hcan1.pTxMsg->RTR     = CAN_RTR_DATA;
    hcan1.pTxMsg->DLC     = 0x08;
    hcan1.pTxMsg->Data[0] = motor1 >> 8;
    hcan1.pTxMsg->Data[1] = motor1;
    hcan1.pTxMsg->Data[2] = motor2 >> 8;
    hcan1.pTxMsg->Data[3] = motor2;
    hcan1.pTxMsg->Data[4] = motor3 >> 8;
    hcan1.pTxMsg->Data[5] = motor3;
    hcan1.pTxMsg->Data[6] = motor4 >> 8;
    hcan1.pTxMsg->Data[7] = motor4;
    HAL_CAN_Transmit(&hcan1, 10);
}

//发送底盘电机控制命令
void send_chassis_cur5_8(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    hcan1.pTxMsg->StdId   = 0x1ff;
    hcan1.pTxMsg->IDE     = CAN_ID_STD;
    hcan1.pTxMsg->RTR     = CAN_RTR_DATA;
    hcan1.pTxMsg->DLC     = 0x08;
    hcan1.pTxMsg->Data[0] = motor5 >> 8;
    hcan1.pTxMsg->Data[1] = motor5;
    hcan1.pTxMsg->Data[2] = motor6 >> 8;
    hcan1.pTxMsg->Data[3] = motor6;
    hcan1.pTxMsg->Data[4] = motor7 >> 8;
    hcan1.pTxMsg->Data[5] = motor7;
    hcan1.pTxMsg->Data[6] = motor8 >> 8;
    hcan1.pTxMsg->Data[7] = motor8;
    HAL_CAN_Transmit(&hcan1, 10);
}

void Lm298n_pin_init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}


void Lm298n_ctrl( int  mode )
{

    if(mode==Lm298n_break)
    {
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
    }
    else if(mode==Lm298n_forward)
    {
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
    }
    else if(mode==Lm298n_backward)
    {
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
    }


}

