#include <adc.h>
#include "usart.h"
#include "tim.h"
#include "gpio.h"

#include <stdio.h>
#include <math.h>

TIM_HandleTypeDef TIM_UTIL_Handler;


uint8_t  analog_data_ready_flag     = 0;
uint16_t analogData[NR_OF_SIGNALS];
uint16_t analog_buffer[NR_OF_KEYS * NR_OF_SIGNALS][AVERAGE_BUFFER_SIZE];
uint8_t  analog_buffer_circular_ptr = 0;


uint32_t first_tactile_ptr       = 0;
uint32_t first_temperature_ptr   = 0;
uint32_t field_config_fulfilment = EEPROM_FIELD_CONFIG_START;
uint8_t  field_config_flag       = 0;

float tactile_angle;
float tactile_force;

extern ADC_HandleTypeDef  ADC_Typedef;
extern UART_HandleTypeDef USB_UART_TypeDef;

/* TIM1 init function */
void MX_TIM1_Init() {
    // 32 MHz input
    // 1Hz output
    TIM_MasterConfigTypeDef        sMasterConfig        = {0};
    TIM_OC_InitTypeDef             sConfigOC            = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    TIM_UTIL_Handler.Instance                           = TIM_UTIL_Instance;
    TIM_UTIL_Handler.Init.Prescaler                     = 32000 - 1;
    TIM_UTIL_Handler.Init.CounterMode                   = TIM_COUNTERMODE_UP;
    // TIM_UTIL_Handler.Init.Period                        = (1000 / TIM_UTIL_FREQ) - 1;
    TIM_UTIL_Handler.Init.Period                        = (1000 / 10) - 1;
    TIM_UTIL_Handler.Init.ClockDivision                 = TIM_CLOCKDIVISION_DIV1;
    TIM_UTIL_Handler.Init.RepetitionCounter             = 0;
    TIM_UTIL_Handler.Init.AutoReloadPreload             = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_OC_Init(&TIM_UTIL_Handler) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&TIM_UTIL_Handler, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode       = TIM_OCMODE_TIMING;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_OC_ConfigChannel(&TIM_UTIL_Handler, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&TIM_UTIL_Handler, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &TIM_UTIL_Handler) {
        take_Measurements();

        // if (!check_Config_Presence()) {
        //     calculate_Tactile_Output();
        //
        //     static uint8_t send_data[12];
        //     // Making sure that force wont get out of range
        //     if (tactile_force > 9999) {
        //         tactile_force = 9999;
        //     }
        //     sprintf((char *) &send_data[0], "%04d,%04d;\r\n", radian2degree(tactile_angle),
        //             (uint16_t) tactile_force);

        // DEBUG tutaj jest kod wysyłający czyste pomiary do użytkownika
        static uint8_t send_data[NR_OF_KEYS * NR_OF_SIGNALS * 5 + 4];

        for (int i = 0; i < NR_OF_KEYS; i++) { // klucze
            for (int j = 0; j < NR_OF_SIGNALS; j++) { // channels
                sprintf((char *) &send_data[5 * (i * NR_OF_SIGNALS + j)], "%04d,",
                        analog_buffer[i * NR_OF_SIGNALS + j][analog_buffer_circular_ptr]);
            }
            sprintf((char *) &send_data[5 * ((i + 1) * NR_OF_SIGNALS) - 1], "|");
        }
        sprintf((char *) &send_data[NR_OF_KEYS * NR_OF_SIGNALS * 5], "\n\r\n");

        HAL_UART_Transmit_IT(&USB_UART_TypeDef, send_data, sizeof(send_data));


        // }
        led_work_toggle();
    }
}

void take_Measurements() {
    for (int i = 0; i < NR_OF_KEYS; i++) {
        keys_set_open(i + 1);
        HAL_ADC_Start_DMA(&ADC_Typedef, (uint32_t *) analogData, NR_OF_SIGNALS);

        while (!analog_data_ready_flag) {
            __NOP();
        }
        // HAL_Delay(100);

        analog_data_ready_flag = 0;
        // TODO czy on zdąża???
        // Kurwa musi xd
        // Tak, ale tylko ~11 pomiarów potem to nawet DMA nie daje rady
        // czekanie nie zmianę dma state nie pomaga
        for (int j = 0; j < NR_OF_SIGNALS; j++) {
            uint16_t value = analogData[j];
            analog_buffer[i * NR_OF_SIGNALS + j][analog_buffer_circular_ptr] = value;
        }
        // Reset of keys at the end of the loop
        keys_all_off();
    }
    analog_buffer_circular_ptr = (analog_buffer_circular_ptr + 1) % AVERAGE_BUFFER_SIZE;
}

uint8_t calculate_Tactile_Output() {
    // TODO Obliczasnie kierunku i mocy wynikowej przykłądanej siły
    // Check if we got config of tactile
    if (first_tactile_ptr == 0) {
        return err_no_tactile_config;
    }

    // Reset tactile sensor values
    uint32_t actual_memory_ptr = first_tactile_ptr;
    tactile_force = 0;
    tactile_angle = 0;
    while (1) {
        // Check if tested field is tactile
        if ((*(uint8_t *) actual_memory_ptr) == FIELD_TYPE_TACTILE_SIDE) {
            uint8_t skip_flag = 0;
            uint8_t key       = (*(uint8_t *) (actual_memory_ptr + 2));
            uint8_t channel   = (*(uint8_t *) (actual_memory_ptr + 3));
            float   pom_angle = degree2radian(*(uint16_t *) (actual_memory_ptr + 4));
            float   pom_dist  = (float) (*(uint16_t *) (actual_memory_ptr + 6));

            // If distance is zero skip field
            if (pom_dist == 0.0f) {
                skip_flag = 1;
            }
            uint16_t pom_average = calculate_average_analog(key * NR_OF_SIGNALS + channel);

            // TODO Do przemyślenia jak to rozwiązać
            // If we got so little signal that we can interpret it as noise
            // skip that measure and go to next one
            if (pom_average > TACT_IGNORANCE_LIMIT) {
                skip_flag = 1;
            }

            if (!skip_flag) {
                float pom_force = calculate_Force(pom_average) / (pom_dist / TACT_FORCE_DIVIDER);

                // Calculate new force vector
                float force_x = pom_force * cosf(pom_angle) + tactile_force * cosf(tactile_angle);
                float force_y = pom_force * sinf(pom_angle) + tactile_force * sinf(tactile_angle);
                tactile_force = sqrtf(force_x * force_x + force_y * force_y);
                tactile_angle = atan2f(force_y, force_x);
            }

            // It was last tactile field - end loop
            if ((*(uint8_t *) (actual_memory_ptr + 1)) == 0) {
                break;
            }
        }

        // Go to next field and check if we are still in range
        actual_memory_ptr += EEPROM_FIELD_SIZE;
        if (actual_memory_ptr >= field_config_fulfilment) {
            return err_try_to_get_outside_of_memory;
        }
    }
    return err_OK;
}

uint8_t check_Config_Presence() {
    if (!field_config_flag) {
        return err_no_config;
    }
    return err_OK;
}

float degree2radian(uint16_t degree) {
    return (float) degree * PI / 180;
}

uint16_t radian2degree(float radian) {
    // Make sure I have positive angle, otherwise was returning bullshit
    while (radian < 0) {
        radian += 2 * PI;
    }

    uint16_t degree = (uint16_t) (radian * 180.0f / PI);
    return degree % 360;
}

float calculate_Force(uint16_t raw_analog) {
    return TACT_MEASURE(raw_analog);
}

uint16_t calculate_average_analog(uint8_t cell) {
    uint16_t sum = 0;
    for (int i   = 0; i < AVERAGE_BUFFER_SIZE; i++) {
        sum += analog_buffer[cell][i];
    }
    return sum / AVERAGE_BUFFER_SIZE;
}

void erase_FLASH(uint32_t start, uint8_t pages) {
    // Erase EEPROM
    uint32_t               page_error = 0;
    FLASH_EraseInitTypeDef s_eraseinit;

    s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
    s_eraseinit.PageAddress = start;
    s_eraseinit.NbPages     = pages; // Erase with place for laser configuration

    HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
}

