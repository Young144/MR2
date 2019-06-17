#ifndef LED_H
#define LED_H
#include "robocon.h"

#define IndicateLED_On HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2,GPIO_PIN_SET)
#define IndicateLED_Off HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2,GPIO_PIN_RESET)

#define IndLED_BLUE HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define IndLED_GREEN HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)
#define IndicateLED_On HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2,GPIO_PIN_SET)


#define IndColorRed 0
#define IndColorBlue 1
#define IndColorGreen 2


void led_configuration(void);

extern void led_green_on(void);
extern void led_green_off(void);
extern void led_green_toggle(void);

extern void led_red_on(void);
extern void led_red_off(void);
extern void led_red_toggle(void);

extern void flow_led_on(uint16_t num);
extern void flow_led_off(uint16_t num);
extern void flow_led_toggle(uint16_t num);

extern void IndLED_On(int color);
extern void IndLED_Off(void);

#endif
