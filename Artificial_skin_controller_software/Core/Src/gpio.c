#include "gpio.h"


void led_Init() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    LED_CLK_ENABLE;

    HAL_GPIO_WritePin(LED_WORK_Port, LED_WORK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_ERROR_Port, LED_ERROR_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = LED_WORK_Pin;
    HAL_GPIO_Init(LED_WORK_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LED_ERROR_Pin;
    HAL_GPIO_Init(LED_ERROR_Port, &GPIO_InitStruct);
}

void led_error_on() {
    HAL_GPIO_WritePin(LED_ERROR_Port, LED_ERROR_Pin, GPIO_PIN_SET);
}

void led_error_off() {
    HAL_GPIO_WritePin(LED_ERROR_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
}

void led_work_on() {
    HAL_GPIO_WritePin(LED_WORK_Port, LED_WORK_Pin, GPIO_PIN_SET);
}

void led_work_toggle() {
    HAL_GPIO_TogglePin(LED_WORK_Port, LED_WORK_Pin);
}

void led_work_off() {
    HAL_GPIO_WritePin(LED_WORK_Port, LED_WORK_Pin, GPIO_PIN_RESET);
}

void keys_Init() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    KEY_CLK_ENABLE;

    HAL_GPIO_WritePin(KEY_1_Port, KEY_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_2_Port, KEY_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_3_Port, KEY_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_4_Port, KEY_4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_5_Port, KEY_5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_6_Port, KEY_6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_7_Port, KEY_7_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_8_Port, KEY_8_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_9_Port, KEY_9_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_10_Port, KEY_10_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_11_Port, KEY_11_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_12_Port, KEY_12_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_13_Port, KEY_13_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_14_Port, KEY_14_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_15_Port, KEY_15_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_16_Port, KEY_16_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = KEY_1_Pin;
    HAL_GPIO_Init(KEY_1_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_2_Pin;
    HAL_GPIO_Init(KEY_2_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_3_Pin;
    HAL_GPIO_Init(KEY_3_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_4_Pin;
    HAL_GPIO_Init(KEY_4_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_5_Pin;
    HAL_GPIO_Init(KEY_5_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_6_Pin;
    HAL_GPIO_Init(KEY_6_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_7_Pin;
    HAL_GPIO_Init(KEY_7_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_8_Pin;
    HAL_GPIO_Init(KEY_8_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_9_Pin;
    HAL_GPIO_Init(KEY_9_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_10_Pin;
    HAL_GPIO_Init(KEY_10_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_11_Pin;
    HAL_GPIO_Init(KEY_11_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_12_Pin;
    HAL_GPIO_Init(KEY_12_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_13_Pin;
    HAL_GPIO_Init(KEY_13_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_14_Pin;
    HAL_GPIO_Init(KEY_14_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_15_Pin;
    HAL_GPIO_Init(KEY_15_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = KEY_16_Pin;
    HAL_GPIO_Init(KEY_16_Port, &GPIO_InitStruct);
}

void keys_all_off() {
    HAL_GPIO_WritePin(KEY_1_Port, KEY_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_2_Port, KEY_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_3_Port, KEY_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_4_Port, KEY_4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_5_Port, KEY_5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_6_Port, KEY_6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_7_Port, KEY_7_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_8_Port, KEY_8_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_9_Port, KEY_9_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_10_Port, KEY_10_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_11_Port, KEY_11_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_12_Port, KEY_12_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_13_Port, KEY_13_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_14_Port, KEY_14_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_15_Port, KEY_15_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_16_Port, KEY_16_Pin, GPIO_PIN_RESET);
}

void keys_set_open(uint8_t key) {
    keys_all_off();

    switch (key) {
        case 1:
            HAL_GPIO_WritePin(KEY_1_Port, KEY_1_Pin, GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(KEY_2_Port, KEY_2_Pin, GPIO_PIN_SET);
            break;
        case 3:
            HAL_GPIO_WritePin(KEY_3_Port, KEY_3_Pin, GPIO_PIN_SET);
            break;
        case 4:
            HAL_GPIO_WritePin(KEY_4_Port, KEY_4_Pin, GPIO_PIN_SET);
            break;
        case 5:
            HAL_GPIO_WritePin(KEY_5_Port, KEY_5_Pin, GPIO_PIN_SET);
            break;
        case 6:
            HAL_GPIO_WritePin(KEY_6_Port, KEY_6_Pin, GPIO_PIN_SET);
            break;
        case 7:
            HAL_GPIO_WritePin(KEY_7_Port, KEY_7_Pin, GPIO_PIN_SET);
            break;
        case 8:
            HAL_GPIO_WritePin(KEY_8_Port, KEY_8_Pin, GPIO_PIN_SET);
            break;
        case 9:
            HAL_GPIO_WritePin(KEY_9_Port, KEY_9_Pin, GPIO_PIN_SET);
            break;
        case 10:
            HAL_GPIO_WritePin(KEY_10_Port, KEY_10_Pin, GPIO_PIN_SET);
            break;
        case 11:
            HAL_GPIO_WritePin(KEY_11_Port, KEY_11_Pin, GPIO_PIN_SET);
            break;
        case 12:
            HAL_GPIO_WritePin(KEY_12_Port, KEY_12_Pin, GPIO_PIN_SET);
            break;
        case 13:
            HAL_GPIO_WritePin(KEY_13_Port, KEY_13_Pin, GPIO_PIN_SET);
            break;
        case 14:
            HAL_GPIO_WritePin(KEY_14_Port, KEY_14_Pin, GPIO_PIN_SET);
            break;
        case 15:
            HAL_GPIO_WritePin(KEY_15_Port, KEY_15_Pin, GPIO_PIN_SET);
            break;
        case 16:
            HAL_GPIO_WritePin(KEY_16_Port, KEY_16_Pin, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}
