#ifndef PostureCtrl_H
#define PostureCtrl_H

#include "robocon.h"

#define L1 10
#define L2 20

#define ReductionAndAngleRatio 436.926337  //3591/187*8191/360=436.926337

void StartPosToMiddlePos(void);


extern float x, y, theta1, theta2;

typedef struct
{
    double L;
    double X;
    double Y;
    double M;
    double N;
    double A1;
    double A2;
    double R[4];

} Coordinate_Trans_Data;

extern Coordinate_Trans_Data coor_calc;

enum States {
    STOP = 0,
		REALSE=1,
    PRONK = 2,
    TROT = 3,
    PACE = 4,
    BOUND = 5,
    WALK_AHEAD=6,
    WALK_BACK=7,
    WALK_LEFT=8,
    WALK_RIGHT=9,
    ROTAT_LEFT=10,
    ROTAT_RIGHT=11,
		JUMP=12,
		START=13,
		END=14
};
extern enum States state;

// 腿部PID增益结构体
typedef struct  {
    float kp_spd;		//速度环
    float kd_spd;

    float kp_pos;		//位置环
    float kd_pos;
} LegGain;

typedef struct  {
    float stance_height ; // 狗身到地面的距离 (cm)
    float step_length ; // 一步的距离 (cm)
    float up_amp ; // 上部振幅y (cm)
    float down_amp ; // 下部振幅 (cm)
    float flight_percent ; // 摆动相百分比 (cm)
    float freq ; // 一步的频率 (Hz)
} GaitParams;

extern GaitParams gait_params;

#endif
