#ifndef __gpio_H
#define __gpio_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


#define LED_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED_WORK_Pin GPIO_PIN_3
#define LED_WORK_Port GPIOB
#define LED_ERROR_Pin GPIO_PIN_4
#define LED_ERROR_Port GPIOB

#define KEY_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE()
#define KEY_1_Pin GPIO_PIN_5
#define KEY_1_Port GPIOB
#define KEY_2_Pin GPIO_PIN_6
#define KEY_2_Port GPIOB
#define KEY_3_Pin GPIO_PIN_7
#define KEY_3_Port GPIOB
#define KEY_4_Pin GPIO_PIN_8
#define KEY_4_Port GPIOB
#define KEY_5_Pin GPIO_PIN_9
#define KEY_5_Port GPIOB
#define KEY_6_Pin GPIO_PIN_13
#define KEY_6_Port GPIOC
#define KEY_7_Pin GPIO_PIN_14
#define KEY_7_Port GPIOC
#define KEY_8_Pin GPIO_PIN_15
#define KEY_8_Port GPIOC
#define KEY_9_Pin GPIO_PIN_12
#define KEY_9_Port GPIOB
#define KEY_10_Pin GPIO_PIN_13
#define KEY_10_Port GPIOB
#define KEY_11_Pin GPIO_PIN_14
#define KEY_11_Port GPIOB
#define KEY_12_Pin GPIO_PIN_15
#define KEY_12_Port GPIOB
#define KEY_13_Pin GPIO_PIN_6
#define KEY_13_Port GPIOC
#define KEY_14_Pin GPIO_PIN_7
#define KEY_14_Port GPIOC
#define KEY_15_Pin GPIO_PIN_8
#define KEY_15_Port GPIOC
#define KEY_16_Pin GPIO_PIN_9
#define KEY_16_Port GPIOC


void led_Init();

void led_error_on();

void led_error_off();

void led_work_on();

void led_work_off();

void led_work_toggle();

void keys_Init();

void keys_all_off();

void keys_set_open(uint8_t key);


#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */
