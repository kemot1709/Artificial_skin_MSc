#include <tim.h>
#include "main.h"
#include "stm32f1xx_it.h"
#include "usart.h"


#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"


extern DMA_HandleTypeDef  hdma_adc1;
extern ADC_HandleTypeDef  hadc1;
extern UART_HandleTypeDef huart3;


/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler() {
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler() {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
    }
#pragma clang diagnostic pop
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler() {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
    }
#pragma clang diagnostic pop
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler() {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
    }
#pragma clang diagnostic pop
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler() {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
    }
#pragma clang diagnostic pop
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler() {
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler() {
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler() {
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler() {
    HAL_IncTick();
    uart_check_timeout();
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles PVD interrupt through EXTI line 16.
  */
void PVD_IRQHandler() {
    HAL_PWR_PVD_IRQHandler();
}

/**
  * @brief This function handles Flash global interrupt.
  */
void FLASH_IRQHandler() {
    HAL_FLASH_IRQHandler();
}

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler() {
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_adc1);
    analog_data_ready_flag = 1;
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler() {
    HAL_ADC_IRQHandler(&hadc1);
}

/**
  * @brief This function handles TIM1 break interrupt.
  */
void TIM1_BRK_IRQHandler() {
    HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler() {
    HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts.
  */
void TIM1_TRG_COM_IRQHandler() {
    HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler() {
    HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler() {
    HAL_UART_IRQHandler(&huart3);
}

// #pragma clang diagnostic pop