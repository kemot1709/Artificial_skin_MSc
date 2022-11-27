#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


#define PI 3.1416f


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern uint32_t first_tactile_ptr;
extern uint32_t first_temperature_ptr;
extern uint32_t field_config_fulfilment;
extern uint8_t field_config_flag;

extern uint8_t analog_data_ready_flag;


void MX_TIM1_Init(void);
void MX_TIM2_Init(void);

void take_Measurements();
float calculate_Force(uint16_t raw_analog);
uint16_t calculate_average_analog(uint8_t cell);

void reset_All_Keys();
uint8_t calculate_Tactile_Output();
uint8_t check_Config_Presence();

float degree2radian(uint16_t degree);
uint16_t radian2degree(float radian);

uint8_t write8_add(uint8_t *buffer);
void erase_FLASH(uint32_t start, uint8_t pages);

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */
