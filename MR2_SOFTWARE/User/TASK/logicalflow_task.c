/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       logicalflow_task.c/h
  * @brief
  ==============================================================================
  **************************** HBUT ROBOCON 2019****************************
  */

#include "logicalflow_task.h"

//		RED_GROUNG		BLUE_GROUNG

bool GROUND_SELECT=RED_GROUNG;  //红蓝场选择

bool restartflag = 0;

/*********传感器端口说明********************
*
*舵机说明：1号（升杆舵机）H:PD12		2号（令牌舵机）G:PD13		3号（解锁舵机）F:PD14
*
*限位开关		重启1 X:PI6		重启2 A:PI0		重启CLIMB C:PH11  重启STA W:PI5		启动B:PH12
*光电开关（挥手开始上坡）光电1 S:PA0 光电2 T:PA1
*
*指示LED Z PI2
**/

/**
*		控制整个比赛的逻辑过程
*/
void LogicalFlow_task(void *pvParameters)
{
    for(;;) {

//        CAN_RoboModule_DRV_Position_Mode(0,1,2000,2150*4*15.15);  //开机自检
//        osDelay(2000);
//        CAN_RoboModule_DRV_Position_Mode(0,1,2000,0);

        while(1)
        {
            vTaskDelay(100);
            if(keyRestart1==0) {//重启1 U:PA2
                while(keyRestart1==0)
                    vTaskDelay(20);

                IndicateLED_Off;
                IndLED_On(IndColorBlue);
                IndLED_On(IndColorRed);
                goto restart1;
            }
            else if(keyRestart2==0) { //重启2 V:PA3
                while(keyRestart2==0)
                    vTaskDelay(20);

                IndicateLED_Off;
                IndLED_On(IndColorBlue);
                IndLED_On(IndColorGreen);
                goto restart2;
            }
            else if(keyRestartclimb==0) //重启climb
            {
                while(keyRestartclimb==0)
                    vTaskDelay(20);

                IndicateLED_Off;
                IndLED_On(IndColorRed);
                IndLED_On(IndColorGreen);
                state = STOP;
                goto restartclimb;
            }

            if(StartFlag == 1&&restartflag == 0) {
                osDelay(100);
                break;
            }
        }


        while((keyInf2!=0)) //等待令牌光电开关触发
            vTaskDelay(500);

        IndLED_Off();

        IndLED_On(IndColorRed);
        vTaskDelay(1500);  				//等待 mr1走
        Servo1_CLOSE;


        pid_spd_out_limit=  2000;

        state = STOP;
        vTaskDelay(800);
        IndLED_Off();

        pid_spd_out_limit=  6000;

        yaw_set=0;  //------航向角设定为0
        LinearCorrection=normal_correction;		//打开直线矫正

        state = TROT;		//小跑步态

        vTaskDelay(2000);  				//等待 不检测到红色启动区域

        OpenMvInspect(openmv_Red);  //检测到红色  开始转弯



        IndLED_On(IndColorRed);  //红灯指示


        if(GROUND_SELECT==RED_GROUNG) {//红蓝场判断 红场
            yaw_set=10;
            _rotate_angle=yaw_set;
            while(imuinfo.ActVal[0]<=(_rotate_angle-4))
                osDelay(50);
        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            yaw_set=-10;
            _rotate_angle=yaw_set;
            while(imuinfo.ActVal[0]>=(_rotate_angle+4))
                osDelay(50);
        }

        IndLED_Off(); //指示灯灭
        OpenMvInspect(openmv_Yellow);  // -------- 等待检测到转弯 黄色 --沙丘

				
				restart1:
        /////restart only
        pid_spd_out_limit=  6000;
       // yaw_set=0;  //------航向角设定为0
        LinearCorrection=normal_correction;		//打开直线矫正
        state = TROT;		//小跑步态
        /////------end
				
				vTaskDelay(1000);
				
				
				
        if(GROUND_SELECT==RED_GROUNG) {//红蓝场判断 红场
            yaw_set=10;
            _rotate_angle=yaw_set;
            state = ROTAT_LEFT;
            while(imuinfo.ActVal[0]<=_rotate_angle)
                osDelay(20);
        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            yaw_set=-10;
            _rotate_angle=yaw_set;
            state = ROTAT_RIGHT;
            while(imuinfo.ActVal[0]>=_rotate_angle)
                osDelay(20);
        }


        //////////////越沙丘//////////////

        IndLED_On(IndColorRed);

        state = STOP;
        vTaskDelay(200);

        state = REALSE;
        vTaskDelay(300);

        pid_spd_out_limit=  6700;

        StartJump(HAL_GetTick()); //第一次起跳
        vTaskDelay(2300);

        state = REALSE;
        StartJump(HAL_GetTick());	//第二次起跳
        vTaskDelay(2300);

        pid_spd_out_limit=  6000;

        IndLED_Off(); //指示灯灭

        //////////////越沙丘结束//////////////



        pid_spd_out_limit=  6000;

        //////////越过台阶之后的直线行走部分
        state=TROT;
        if(GROUND_SELECT==RED_GROUNG) {  //红蓝场判断 红场
            yaw_set=14.5;
            _rotate_angle=yaw_set;
            LinearCorrection=normal_correction;		//打开直线矫正
            while(imuinfo.ActVal[0]<=(_rotate_angle-1.1))  //快速转过来
                osDelay(50);

            yaw_set=13.6;  //设定角度为13
        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            yaw_set=-14.5;
            _rotate_angle=yaw_set;
            LinearCorrection=normal_correction;		//打开直线矫正
            while(imuinfo.ActVal[0]>=(_rotate_angle+1.1))
                osDelay(50);

            yaw_set=-13.6;
        }
        //////////直线行走部分完



        OpenMvInspect(openmv_Yellow); //等待检测到黄色 色块----------越绳子检测
        IndLED_On(IndColorRed);  //指示灯开


        //////到达黄色区域之后转弯动作
        if(GROUND_SELECT==RED_GROUNG) {  //红蓝场判断 红场
            yaw_set=0;
            _rotate_angle=yaw_set;
            while(imuinfo.ActVal[0]>=(_rotate_angle+5.5))
                osDelay(20);
        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            yaw_set=0;
            _rotate_angle=yaw_set;
            while(imuinfo.ActVal[0]<=(_rotate_angle-5.5))
                osDelay(20);
        }
        IndLED_Off();


        if(GROUND_SELECT==RED_GROUNG) {  //红蓝场判断 红场
            yaw_set=0;
            _rotate_angle=yaw_set;
            state = ROTAT_RIGHT;
            while(imuinfo.ActVal[0]>=(_rotate_angle+0.5))
                osDelay(20);

        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            yaw_set=0;
            _rotate_angle=yaw_set;
            state = ROTAT_LEFT;
            while(imuinfo.ActVal[0]<=(_rotate_angle-0.5))
                osDelay(20);

        }
        //////转弯动作结束


restart2:

        pid_spd_out_limit=  6000;
        state=TROT;
        yaw_set=0.0;
        LinearCorrection=normal_correction;		//打开直线矫正
        OpenMvInspect(openmv_Yellow);

				
        state=STOP;
        vTaskDelay(500);

//				state=WALK_BACK;
//				 vTaskDelay(1000);

//        state=STOP;
//        vTaskDelay(500);


        ////起跳动作组合
        pid_spd_out_limit=  6700;

        jump_angle=45.5;
        jump_extension=28.8;

        state = REALSE;
        StartJump(HAL_GetTick());	//第一次起跳
        vTaskDelay(2200);


        //////////第一次起跳之后调整自己的姿态
        if(imuinfo.ActVal[0]>=0) {
            yaw_set=0-0;
            _rotate_angle=yaw_set;
            state = ROTAT_RIGHT;
            while(imuinfo.ActVal[0]>=(_rotate_angle+1.2))
                osDelay(10);
        }
        else if(imuinfo.ActVal[0]<=0) {
            yaw_set=0-0;
            _rotate_angle=yaw_set;
            state = ROTAT_LEFT;
            while(imuinfo.ActVal[0]<=(_rotate_angle-1.2))
                osDelay(10);
        }
        //////////姿态调整结束


        state = REALSE;
        StartJump(HAL_GetTick());	//第二次起跳
        vTaskDelay(2100);

        state = REALSE;
        StartJump(HAL_GetTick());	//第三次次起跳
        vTaskDelay(2100);


        state = REALSE;
        StartJump(HAL_GetTick());	//第三次次起跳
        vTaskDelay(2100);


        pid_spd_out_limit=  6700;


        //跨绳子传统步态   时间20s
        //	CrossTheLine();
        //	yaw_set=0-0; //航向角设定 0 补偿-0
//        	state=TEST1;
//        	vTaskDelay(1000);


restartclimb:

        state= STOP;

        //////////上坡
        Climbing_Comb();
        //////////上坡完毕

        while(1)		//停
            vTaskDelay(500);


    }
}
