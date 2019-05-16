#ifndef NAVIGATION_H
#define NAVIGATION_H
#include "robocon.h"

#define climbing_correction 4
#define test1_correction 3
#define normal_correction 2


#define Permit 1
#define Deny 0

extern float step_len_dev;
extern float step_high_dev;
extern float step_len_initial;
extern float step_high_initial;
extern float flight_percent_dev;

extern float yaw_now;
extern float yaw_set;

extern float roll_offset;
extern float pitch_offset;

extern int LinearCorrection;
extern bool BalanceCorrection;

extern float _dev_angel;
extern int stage;
extern int _count_navi;

void navigation_execute(void);
#endif
