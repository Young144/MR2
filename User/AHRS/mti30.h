
 
#ifndef _MTI30_H
#define _MTI30_H
	
#include "robocon.h"



#define KALMAN_Q 8

#define KALMAN_R 1.0000



float KalmanFilter( float ResrcData,float ProcessNiose_Q,float MeasureNoise_R);
	
#endif

