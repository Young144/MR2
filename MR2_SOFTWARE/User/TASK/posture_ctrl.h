#ifndef PostureCtrl_H
#define PostureCtrl_H

#include "robocon.h"

#define L1 10
#define L2 20

#define ReductionAndAngleRatio 436.926337  //3591/187*8191/360=436.926337

#define YES 1
#define NO 0

extern float _climbing_offset_angle;
extern bool climbing_offset_flag;

extern const float TROT_step_length2;
extern const float TROT_up_amp2;

extern const float TROT_stance_height;
extern const float TROT_step_length;
extern const float TROT_up_amp;
extern const float TROT_down_amp;
extern const float TROT_flight_percent;
extern const float TROT_freq;
extern bool _leg_active[4];
extern bool rc_ctrl_flag;

extern float x, y, theta1, theta2;

extern float _angle_initial,_rotate_angle;

extern bool TurnLeftFlag1,TurnLeftFlag2,TurnRightFlag1,TurnRightFlag2,ClimbingFlag1,ClimbingFlag2;

extern float now_time;


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

    TROT = 0,
    TEST1=1,
    TEST2=2,
    TEST3=3,
    TEST4=4,
    TEST5=5,
    TEST6=6,
    TEST7=7,
    TEST8=8,
    TEST9=9,
    TEST10=10,
    TEST11=11,
    TEST12=12,
    CLIMBING = 13,


    WALK=14,
    WALK_BACK=15,
    ROTAT_LEFT=16,
    ROTAT_RIGHT=17,


    BOUND=18,	// 跳跑
    GALLOP=19, // 袭步



    STOP = 20,
    REALSE=21,
    JUMP=22,
    START=23,
    END=24

};
extern enum States state;


typedef struct  {		// 腿部PID增益结构体

    float kp_pos;		//位置环
    float kd_pos;

    float kp_spd;		//速度环
    float kd_spd;

} LegGain;
extern LegGain test_gait_gains;

typedef struct  {		// 腿部参数结构体
    float stance_height ; // 狗身到地面的距离 (cm)
    float step_length ; // 一步的距离 (cm)
    float up_amp ; // 上部振幅y (cm)
    float down_amp ; // 下部振幅 (cm)
    float flight_percent ; // 摆动相百分比 (cm)
    float freq ; // 一步的频率 (Hz)
} GaitParams;
extern GaitParams gait_params;
extern GaitParams state_gait_params[];

typedef struct {

    GaitParams detached_params_0;
    GaitParams detached_params_1;

    GaitParams detached_params_2;
    GaitParams detached_params_3;

} DetachedParam;
extern DetachedParam detached_params;
extern DetachedParam state_detached_params[];

extern DetachedParam  RcDetachedParam;

void gait(	GaitParams params,LegGain gait_gains,
            float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
            float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction);
void gait_detached(	DetachedParam d_params,
                    float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
                    float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction);
void CoupledMoveLeg(float t, GaitParams params,float gait_offset, float leg_direction, int LegId);
void CycloidTrajectory (float t, GaitParams params, float gait_offset) ;
void SinTrajectory (float t, GaitParams params, float gait_offset) ;
void CartesianToTheta(float leg_direction) ;
void SetCoupledPosition(int LegId);
void CommandAllLegs(LegGain gains);
bool IsValidLegGain( LegGain gains);
bool IsValidGaitParams( GaitParams params);
void ChangeTheGainsOfPD(LegGain gains);
void RenewYaw (void);
void CommandAllLegs_v(void);
#endif
