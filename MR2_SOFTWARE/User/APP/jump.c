/**
  ****************************(C) COPYRIGHT DJI****************************
  * @file       jump.c/h
  * @brief      跳跃实现函数
  ==============================================================================
  **************************** HBUT ROBOCON 2019****************************
  */

#include "jump.h"
float start_time_ = 0.0f;

float jump_angle = 31;

float jump_extension = 28.8f; // 最大伸腿长度 [cm]

void TrajectoryJump(float t, float launchTime, float stanceHeight, float downAMP) {
    //Need to check if n works
    float n = t/launchTime;
    x = 0;
    y = downAMP*n + stanceHeight;
    //y = downAMP*sin(PI/4 + PI/4*n) + stanceHeight;
}

void StartJump(float start_time_s) {
    start_time_ = start_time_s;
    state = JUMP;
}

void ExecuteJump() {
    // min radius = 0.8
    // max radius = 0.25

    const float prep_time = 0.8f; // 准备时间 [s]		0.8
    const float launch_time = 0.2f ; // 收缩腿前的持续时间 [s]		0.2
    const float fall_time = 0.8f; //收缩腿后持续时间恢复正常行为 [s]		0.8

    const float stance_height = 15.8f; // 跳跃之前腿的高度 [cm]  14.2
    const float fall_extension = 15.8f; // 降落时的期望腿长 [cm]

//	// -----------------    用腿定位-------------///
//	 x = -jump_extension*sin(45*PI/180);
//        y = jump_extension*cos(45*PI/180);
//        CartesianToTheta(1.0);
//        CommandAllLegs_v();
//				vTaskDelay(200);
//

    float t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间

    if (t < prep_time) {

        x = -stance_height*sin(21*PI/180);
        y = stance_height*cos(21*PI/180);
//        x = 0;
//        y = stance_height;
        CartesianToTheta(1.0);
        // 使用刚度小，阻尼大的增益
        LegGain gains = {5, 0.00, 8, 0.00};
        //CommandAllLegs_v();
        CommandAllLegs(gains);

    } else if (t >= prep_time && t < prep_time + launch_time) {

        x = -jump_extension*sin(jump_angle*PI/180);
        y = jump_extension*cos(jump_angle*PI/180);
//        x = 0;
//        y = jump_extension;

        CartesianToTheta(1.0);

        // 使用高刚度和低阻尼执行跳转
        LegGain gains = {22.0, 0.00, 16.0, 0.00};
        //CommandAllLegs_v();
        CommandAllLegs(gains);

    } else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time) {

        x = 0;
        y = fall_extension;
        CartesianToTheta(1.0);
        //使用低刚度和大量的阻尼来处理下降
        LegGain gains = {15, 0.00, 1.8, 0.00};  //前位置环 后速度环

        CommandAllLegs(gains);

    } else {

        //LegGain gains = {22.0, 0.00, 8.0, 0.00};
        state = STOP;
        printf("Jump Complete.\r\n");
    }

}

