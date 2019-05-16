/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器通过DMA串口接收数据，利用串口空闲中断来拉起处
  *            	理函数
  ==============================================================================
  **************************** HBUT ROBOCON 2019****************************
  */

#include "Remote_Control.h"


void Rc_task(void *pvParameters)
{

    while(!imu_rec_flag)
        vTaskDelay(500);

    ActionDoneBuzzer();



    for(;;)

    {


        switch(ps2info.KEY_VALUE)
        {
        case PSB_PAD_UP:
//            now_time=times;
//            yaw_set=0;
// 						LinearCorrection=Permit;		//打开直线矫正
            state = TROT;
            break;

        case PSB_PAD_DOWN:
            state = WALK_BACK;
            break;

        case PSB_PAD_LEFT:
            yaw_set++;
            if(yaw_set>179.9)
                yaw_set=-179.9;
            break;

        case PSB_PAD_RIGHT:
            yaw_set--;
            if(yaw_set<-179.9)
                yaw_set=179.9;
            break;

        case PSB_SQUARE:
//            _angle_initial=imuinfo.ActVal[0];
//            _rotate_angle=45;
            state = ROTAT_LEFT;
            break;

        case PSB_CIRCLE:
//            _angle_initial=imuinfo.ActVal[0];
//            _rotate_angle=45;
            state = ROTAT_RIGHT;
            break;

        case PSB_R1:
            LinearCorrection=Deny;
            BalanceCorrection=Deny;
            stage=0;
            state = STOP;
            break;

        case PSB_R2:
//            TurnLeftFlag1=YES;
            break;

        case PSB_R3:
            state = TEST6;
            break;

        case PSB_SELECT:
            state = TEST12;
            //StartJump(times);
            break;

        case PSB_L1:
            state = START;
            break;

        case PSB_L2:

            state = END;
            break;

        case PSB_L3:
//            _dev_angel=imuinfo.ActVal[0];
            //LinearCorrection=Permit;
            yaw_set=0;
            state = CLIMBING;
            break;



        }

        vTaskDelay(200);

    }

}
