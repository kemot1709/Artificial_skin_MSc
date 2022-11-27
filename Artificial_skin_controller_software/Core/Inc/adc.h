#ifndef __adc_H
#define __adc_H
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


#define ADC_DMA_Typedef hdma_adc1
#define ADC_DMA_Instance DMA1_Channel1
#define ADC_DMA_CLK_ENABLE __HAL_RCC_DMA1_CLK_ENABLE()
#define ADC_DMA_IRQ DMA1_Channel1_IRQn

#define ADC_Typedef hadc1
#define ADC_Instance ADC1
#define ADC_IRQ ADC1_2_IRQn
#define ADC_CLK_ENABLE __HAL_RCC_ADC1_CLK_ENABLE()
#define ADC_GPIO_CLK_ENABLE __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE()

#define SGN_1_Pin GPIO_PIN_0
#define SGN_1_Port GPIOC
#define SGN_1_CHANNEL ADC_CHANNEL_10
#define SGN_1_RANK ADC_REGULAR_RANK_1

#define SGN_2_Pin GPIO_PIN_1
#define SGN_2_Port GPIOC
#define SGN_2_CHANNEL ADC_CHANNEL_11
#define SGN_2_RANK ADC_REGULAR_RANK_2

#define SGN_3_Pin GPIO_PIN_2
#define SGN_3_Port GPIOC
#define SGN_3_CHANNEL ADC_CHANNEL_12
#define SGN_3_RANK ADC_REGULAR_RANK_3

#define SGN_4_Pin GPIO_PIN_3
#define SGN_4_Port GPIOC
#define SGN_4_CHANNEL ADC_CHANNEL_13
#define SGN_4_RANK ADC_REGULAR_RANK_4

#define SGN_5_Pin GPIO_PIN_0
#define SGN_5_Port GPIOA
#define SGN_5_CHANNEL ADC_CHANNEL_0
#define SGN_5_RANK ADC_REGULAR_RANK_5

#define SGN_6_Pin GPIO_PIN_1
#define SGN_6_Port GPIOA
#define SGN_6_CHANNEL  ADC_CHANNEL_1
#define SGN_6_RANK ADC_REGULAR_RANK_6

#define SGN_7_Pin GPIO_PIN_2
#define SGN_7_Port GPIOA
#define SGN_7_CHANNEL ADC_CHANNEL_2
#define SGN_7_RANK ADC_REGULAR_RANK_7

#define SGN_8_Pin GPIO_PIN_3
#define SGN_8_Port GPIOA
#define SGN_8_CHANNEL ADC_CHANNEL_3
#define SGN_8_RANK ADC_REGULAR_RANK_8

#define SGN_9_Pin GPIO_PIN_4
#define SGN_9_Port GPIOA
#define SGN_9_CHANNEL ADC_CHANNEL_4
#define SGN_9_RANK ADC_REGULAR_RANK_9

#define SGN_10_Pin GPIO_PIN_5
#define SGN_10_Port GPIOA
#define SGN_10_CHANNEL ADC_CHANNEL_5
#define SGN_10_RANK ADC_REGULAR_RANK_10

#define SGN_11_Pin GPIO_PIN_6
#define SGN_11_Port GPIOA
#define SGN_11_CHANNEL ADC_CHANNEL_6
#define SGN_11_RANK ADC_REGULAR_RANK_11

#define SGN_12_Pin GPIO_PIN_7
#define SGN_12_Port GPIOA
#define SGN_12_CHANNEL ADC_CHANNEL_7
#define SGN_12_RANK ADC_REGULAR_RANK_12

#define SGN_13_Pin GPIO_PIN_4
#define SGN_13_Port GPIOC
#define SGN_13_CHANNEL ADC_CHANNEL_14
#define SGN_13_RANK ADC_REGULAR_RANK_13

#define SGN_14_Pin GPIO_PIN_5
#define SGN_14_Port GPIOC
#define SGN_14_CHANNEL ADC_CHANNEL_15
#define SGN_14_RANK ADC_REGULAR_RANK_14

#define SGN_15_Pin GPIO_PIN_0
#define SGN_15_Port GPIOB
#define SGN_15_CHANNEL ADC_CHANNEL_8
#define SGN_15_RANK ADC_REGULAR_RANK_15

#define SGN_16_Pin GPIO_PIN_1
#define SGN_16_Port GPIOB
#define SGN_16_CHANNEL ADC_CHANNEL_9
#define SGN_16_RANK ADC_REGULAR_RANK_16


void signal_adc_Init();

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

