
#ifndef __VCAN_H
#define __VCAN_H

#include "robocon.h"
//void vcan_send_byte(uint8_t date);
void Vcan_Send_Wave_Data(void);
void VcanGC_task(void *pvParameters);
extern short  wave_form_data[8] ;
#endif

