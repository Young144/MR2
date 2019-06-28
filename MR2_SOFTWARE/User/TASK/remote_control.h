#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "robocon.h"

/* ----------------------- RC PPM DEFINE ----------------------------- */
#define RC_SWITCH_ON
extern float step_len_throttle;
extern float step_len_yaw;

extern float step_len_rotate_angle;

extern bool StartFlag;

/* ----------------------- RC Switch Definition----------------------------- */

/* ----------------------- PC Key Definition-------------------------------- */

#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8

#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12

#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16


/* ----------------------- Internal Data ----------------------------------- */

#endif
