#include "iwdg.h"

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void IWDG_Init() {
    // LSI_VALUE = 40kHz
    // Reset 1s
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload    = 625 - 1;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

void IWDG_Refresh() {
    HAL_IWDG_Refresh(&hiwdg);
}


