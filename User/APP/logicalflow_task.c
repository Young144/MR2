/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       logicalflow_task.c/h
  * @brief      
  ==============================================================================
  **************************** HBUT ROBOCON 2019****************************
  */

#include "logicalflow_task.h"


/*********传感器端口说明********************
*
*舵机说明：1号（升杆舵机）H:PD12		2号（令牌舵机）G:PD13		3号（解锁舵机）F:PD14
*
*限位开关 令牌检测A:PI0		启动B:PH12		重启1 U:PA2		重启2 V:PA3		(重启3 W:PI5)	占时没有用到 chongqi climb C : PH11
*光电开关（挥手开始上坡）光电1 S:PA0 光电2 T:PA1
*
*指示LED Z PI2
**/
//#define keyToken HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_0)
//#define keyStart HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_12)

//#define keyRestart1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)
//#define keyRestart2 HAL_GPIO_ReadPin(GPIOA,GPazIO_PIN_3)
//#define keyRestart3 HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5)
//#define keyRestartclimb HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_11)

//#define keyInf1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)
//#define keyInf2 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)

//#define RED_GROUNG 0
//#define BLUE_GROUNG 1

//bool GROUND_SELECT=BLUE_GROUNG;  //红蓝场选择

/**
*		控制整个比赛的逻辑过程 逻辑流控制
*/
void LogicalFlow_task(void *pvParameters)
{
    for(;;) {

        IndicateLED_Off;

        //KeyToken_Test();

        while(1)
        {
            vTaskDelay(500);
            if(keyStart==0)//等待限位开关按下去 启动B:PH12
            {
                state = START;
                break;
            }
            else if(keyRestart1==0) //重启1 U:PA2
                goto restart1;

            else if(keyRestart2==0) //重启2 V:PA3
                goto restart2;

            else if(keyRestartclimb==0) //重启climb
						{
							Servo3_CLOSE;
							Servo2_DOWN_POS;
							state = STOP;
							    state_detached_params[CLIMBING].detached_params_0.stance_height=16;
    state_detached_params[CLIMBING].detached_params_1.stance_height=16;
                goto restartclimb;
						}
        }


        while((keyInf2!=0)) //等待限位开关按下去 令牌落下 A  PI0
            vTaskDelay(500);

        IndicateLED_On;

        vTaskDelay(1000);  				//等待 mr1走



        //Climbing_Comb();  //上坡测试程序

restart1:

        state = STOP;
        vTaskDelay(800);

        IndicateLED_Off;

        yaw_set=0;  //------航向角设定为0
        LinearCorrection=normal_correction;		//打开直线矫正
        //now_time=times;
        state = TROT;		//小跑步态


        vTaskDelay(1000);

        OpenMvInspect(openmv_Yellow);  // -------- 等待检测到转弯 黄色 --沙丘

        IndicateLED_On;

        state= STOP ;
        vTaskDelay(200);

        if(GROUND_SELECT==RED_GROUNG) {//红蓝场判断 红场
            _rotate_angle=10;
            yaw_set=10;		//----------------------航向角设定 10 补偿0
            //now_time=times;
            state = TEST7;  //--------小碎步转弯 左
            while(imuinfo.ActVal[0]<=_rotate_angle)		//转到45度之后自动停止 10
                osDelay(50);
        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            _rotate_angle=-10;
            yaw_set=-10;		//----------------------航向角设定 -10 补偿0
            now_time=times;
            state = TEST8;  //--------小碎步转弯 右
            while(imuinfo.ActVal[0]>=_rotate_angle)		//转到45度之后自动停止 10
                osDelay(50);
        }

        IndicateLED_Off;

        state=TEST6;				// --垮沙丘前  小碎步走一走

        OpenMvInspect(openmv_Yellow);

        vTaskDelay(1000);
        state=REALSE;
        vTaskDelay(400);

        //---------------------越沙丘---------------------//
        StepOver();

        // now_time=times;
        state=TROT;		    //-------------------小跑跑下沙丘
        vTaskDelay(800);

        state=REALSE;
        vTaskDelay(250);
        state=STOP;
        vTaskDelay(500);

        if(GROUND_SELECT==RED_GROUNG) {  //红蓝场判断 红场
            _rotate_angle=14;		//---------------------转向
            yaw_set=14;  //----------------航向角设定 14 补偿0
            LinearCorrection=normal_correction;		//打开直线矫正
            now_time=times;
            state = ROTAT_LEFT;
            while(imuinfo.ActVal[0]<=_rotate_angle)
                osDelay(50);
        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            _rotate_angle=-14;		//---------------------转向
            yaw_set=-14;  //----------------航向角设定 14 补偿0
            LinearCorrection=normal_correction;		//打开直线矫正
            now_time=times;
            state = ROTAT_RIGHT;
            while(imuinfo.ActVal[0]>=_rotate_angle)
                osDelay(50);
        }

restart2:

        state= STOP;
        vTaskDelay(400);


        if(GROUND_SELECT==RED_GROUNG)  //红蓝场判断 红场
            yaw_set=14;  //----------------航向角设定 14 补偿0
        else if(GROUND_SELECT==BLUE_GROUNG)
            yaw_set=-14;  //----------------航向角设定 14 补偿0

        LinearCorrection=normal_correction;		//打开直线矫正
        now_time=times;

        state= TROT;		//继续走直到转弯检测


        vTaskDelay(1600);


        OpenMvInspect(openmv_Yellow); //等待检测到黄色 色块----------越绳子检测

        IndicateLED_On;

        state=REALSE;

        if(GROUND_SELECT==RED_GROUNG) {  //红蓝场判断 红场
            _rotate_angle=0-0;		//-------------转向-----------正对绳子-------
            yaw_set=0-0;
            state = ROTAT_RIGHT;
            while(imuinfo.ActVal[0]>=_rotate_angle)
                osDelay(20);
        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            _rotate_angle=0+0;		//-------------转向-----------正对绳子-------
            yaw_set=0+0;
            state = ROTAT_LEFT;
            while(imuinfo.ActVal[0]<=_rotate_angle)
                osDelay(20);
        }

        state=REALSE;

        IndicateLED_Off;

        state=TEST6;				//小碎步
        vTaskDelay(2500);

        state=REALSE;

        //---------------------跨绳子---------------------//
        LinearCorrection=test1_correction;		//打开跨绳子时候的步态纠偏
        CrossTheLine();

        yaw_set=0-0; //航向角设定 0 补偿-0
        now_time=times;
        state=TEST1;
        vTaskDelay(1500);

restartclimb:
				
        state= STOP;

        //...上坡------------
        Climbing_Comb();


        while(1)		//停
            vTaskDelay(500);


    }
}
