
#include "moto_ctrl.h"

void send_chassis_cur1_4(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void send_chassis_cur5_8(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);

bool IsMotoReadyOrNot= NotReady;
float ref_agle[8]= {0};
float temp_angle;
temp_data temp_pid= {0};     //pid中间数据

float test_speed=0;
/*******************************************************************************************
	*@ 函数名称：void MotorControl_task(void *pvParameters)
	*@ 功能： 接受电机反馈数据并且进行PID计算 输出电流大小控制电机位置
	*@ 备注： FREERTOS任务函数
 *******************************************************************************************/
void MotorControl_task(void *pvParameters)
{
    for(;;)
        moto_behaviour();
}

/**
* NAME: void moto_behaviour(void)
* FUNCTION : 电机pid行为
*/
void moto_behaviour(void)
{

    if(IsMotoReadyOrNot== IsReady) {
        for(int i=0; i<8; i++)
            ref_agle[i]=temp_pid.ref_agle[i];
        IsMotoReadyOrNot= NotReady;
    }

    for(int i=0; i<8; i++)
    {
			//  pid_calc(&pid_pos[i],moto_chassis[i].total_angle/100,ref_agle[i]/100);  //位置环 角度控制
        pid_calc(&pid_pos[i],moto_chassis[i].total_angle/100,ref_agle[i]/100);  //位置环 角度控制
      //  temp_pid.out[i] = pid_calc(&pid_spd[i],moto_chassis[i].speed_rpm,pid_pos[i].pos_out);  //速度环 速度控制
//			  temp_pid.out[i] = pid_calc(&pid_spd[i],moto_chassis[i].speed_rpm,test_speed);  //速度环 速度控制
			  temp_pid.out[i] = pid_calc(&pid_spd[i],moto_chassis[i].speed_rpm,pid_pos[i].pos_out);  //速度环 速度控制
    }

    send_chassis_cur1_4(temp_pid.out[0],temp_pid.out[1],temp_pid.out[2],temp_pid.out[3]);		//传递1-4数据给can收发器
    send_chassis_cur5_8(temp_pid.out[4],temp_pid.out[5],temp_pid.out[6],temp_pid.out[7]);		//传递5-8数据给can收发器

    vTaskDelay(2);		//控制采样频率  实测给 1     7号和8号电机会失控

}

/**
* NAME: void pid_param_init(void)
* FUNCTION : pid参数填充初始化
*/
void pid_param_init(void)
{

    for (int i = 0; i < 8; i++)//最大50 不然发散  						20,0.01,0  37,0.008
        PID_struct_init(&pid_pos[i], POSITION_PID, 100000, 20000, 47,0.008, 0);  //位置环PID参数设置（pid结构体，PID类型，最大输出，比例限制，P , I , D ）

	
	
	
    for (int i = 0; i < 8; i++)  //									16384.0f对应20A					15.5f,0,0 // 16.0f, 0.001f   20.0f, 0.004297f
        PID_struct_init(&pid_spd[i], POSITION_PID, 16384.0f, 2000.0f, 22.0f, 0.01399f, 0.0f);		//速度环PID（pid结构体，PID类型，最大输出，比例限制，P , I , D ）

	
	
	
	
    PID_struct_init(&pid_imu[0], POSITION_PID, 10000, 0, 1.5f, 0.01, 0);		//roll

    PID_struct_init(&pid_imu[1], POSITION_PID, 10000, 0, 1.5f, 0.01, 0);		//pitch

    PID_struct_init(&pid_imu[2], POSITION_PID, 10000, 0, 2.5f, 0, 0);		//yaw

    PID_struct_init(&pid_climbing, POSITION_PID, 10000, 0, 3.5f, 0, 0);		//yaw

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


