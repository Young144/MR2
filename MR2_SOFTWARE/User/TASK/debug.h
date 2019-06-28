#ifndef DEBUG_H
#define DEBUG_H

#include "robocon.h"
//void debug_behaviour(void);
//void calc_transfer_behaviour(void);

extern uint8_t Ready_Flag;
extern uint16_t USART_RX_BUF[];
extern uint8_t act_flag;

extern TaskHandle_t MotorControlTask_Handler;
extern int temp;


void InterpretCommand(void);
void debug_sort(void);
float decimal_converter(void)	 ;
uint16_t get_result_digitals(void);
#endif
