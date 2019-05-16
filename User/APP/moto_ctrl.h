#ifndef MOTO_CTRL_H
#define MOTO_CTRL_H

#include "robocon.h"

#define IsReady  1
#define NotReady 0

void moto_behaviour(void);
void pid_param_init(void);
void coordinate_trans(float X,float Y);

typedef struct
{
    float ref_agle[8];
    float out[8];
} temp_data;


extern bool IsMotoReadyOrNot;
extern float ref_agle[8];
extern temp_data temp_pid;
#endif
