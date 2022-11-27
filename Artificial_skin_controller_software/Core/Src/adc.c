#include "adc.h"


ADC_HandleTypeDef ADC_Typedef;
DMA_HandleTypeDef ADC_DMA_Typedef;


void signal_adc_Init() {
    GPIO_InitTypeDef       GPIO_InitStruct = {0};
    ADC_ChannelConfTypeDef sConfig         = {0};

    ADC_GPIO_CLK_ENABLE;
    ADC_DMA_CLK_ENABLE;
    ADC_CLK_ENABLE;


    HAL_GPIO_WritePin(SGN_1_Port, SGN_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_2_Port, SGN_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_3_Port, SGN_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_4_Port, SGN_4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_5_Port, SGN_5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_6_Port, SGN_6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_7_Port, SGN_7_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_8_Port, SGN_8_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_9_Port, SGN_9_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_10_Port, SGN_10_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_11_Port, SGN_11_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_12_Port, SGN_12_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_13_Port, SGN_13_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_14_Port, SGN_14_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_15_Port, SGN_15_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SGN_16_Port, SGN_16_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

    GPIO_InitStruct.Pin = SGN_1_Pin;
    HAL_GPIO_Init(SGN_1_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_2_Pin;
    HAL_GPIO_Init(SGN_2_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_3_Pin;
    HAL_GPIO_Init(SGN_3_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_4_Pin;
    HAL_GPIO_Init(SGN_4_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_5_Pin;
    HAL_GPIO_Init(SGN_5_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_6_Pin;
    HAL_GPIO_Init(SGN_6_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_7_Pin;
    HAL_GPIO_Init(SGN_7_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_8_Pin;
    HAL_GPIO_Init(SGN_8_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_9_Pin;
    HAL_GPIO_Init(SGN_9_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_10_Pin;
    HAL_GPIO_Init(SGN_10_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_11_Pin;
    HAL_GPIO_Init(SGN_11_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_12_Pin;
    HAL_GPIO_Init(SGN_12_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_13_Pin;
    HAL_GPIO_Init(SGN_13_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_14_Pin;
    HAL_GPIO_Init(SGN_14_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_15_Pin;
    HAL_GPIO_Init(SGN_15_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SGN_16_Pin;
    HAL_GPIO_Init(SGN_16_Port, &GPIO_InitStruct);


    ADC_DMA_Typedef.Instance                 = ADC_DMA_Instance;
    ADC_DMA_Typedef.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    ADC_DMA_Typedef.Init.PeriphInc           = DMA_PINC_DISABLE;
    ADC_DMA_Typedef.Init.MemInc              = DMA_MINC_ENABLE;
    ADC_DMA_Typedef.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    ADC_DMA_Typedef.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    ADC_DMA_Typedef.Init.Mode                = DMA_NORMAL;
    ADC_DMA_Typedef.Init.Priority            = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&ADC_DMA_Typedef) != HAL_OK) {
        Error_Handler();
    }


    ADC_Typedef.Instance                   = ADC_Instance;
    ADC_Typedef.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    ADC_Typedef.Init.ContinuousConvMode    = DISABLE;
    ADC_Typedef.Init.DiscontinuousConvMode = DISABLE;
    ADC_Typedef.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    ADC_Typedef.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    ADC_Typedef.Init.NbrOfConversion       = 16;
    if (HAL_ADC_Init(&ADC_Typedef) != HAL_OK) {
        Error_Handler();
    }


    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;

    sConfig.Channel = SGN_1_CHANNEL;
    sConfig.Rank    = SGN_3_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_2_CHANNEL;
    sConfig.Rank    = SGN_4_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_3_CHANNEL;
    sConfig.Rank    = SGN_5_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_4_CHANNEL;
    sConfig.Rank    = SGN_6_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_5_CHANNEL;
    sConfig.Rank    = SGN_7_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_6_CHANNEL;
    sConfig.Rank    = SGN_8_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_7_CHANNEL;
    sConfig.Rank    = SGN_9_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_8_CHANNEL;
    sConfig.Rank    = SGN_10_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_9_CHANNEL;
    sConfig.Rank    = SGN_11_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_10_CHANNEL;
    sConfig.Rank    = SGN_12_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_11_CHANNEL;
    sConfig.Rank    = SGN_13_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_12_CHANNEL;
    sConfig.Rank    = SGN_14_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_13_CHANNEL;
    sConfig.Rank    = SGN_15_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_14_CHANNEL;
    sConfig.Rank    = SGN_16_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_15_CHANNEL;
    sConfig.Rank    = SGN_1_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = SGN_16_CHANNEL;
    sConfig.Rank    = SGN_2_RANK;
    if (HAL_ADC_ConfigChannel(&ADC_Typedef, &sConfig) != HAL_OK) {
        Error_Handler();
    }


    __HAL_LINKDMA(&ADC_Typedef, DMA_Handle, ADC_DMA_Typedef);

    HAL_NVIC_SetPriority(ADC_DMA_IRQ, 11, 0);
    HAL_NVIC_EnableIRQ(ADC_DMA_IRQ);
    HAL_NVIC_SetPriority(ADC_IRQ, 10, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQ);
}
