#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

// TODO HARDWARE zrobić i pobawić się z hardware USB, a nie przez CP2102
// TODO Ustlić priorytety przerwań w main h

enum {
    err_OK                           = 0x00,
    err_other                        = 0x01,
    err_wtf                          = 0x02,
    err_no_tactile_config            = 0x10,
    err_no_config                    = 0xA0,
    err_try_to_get_outside_of_memory = 0xA1,
};

// #define WATCHDOG_ON

#define SYS_CLK 32000000U
#define ADC_CLK 8000000U

#define EEPROM_START 0x08000000
#define EEPROM_END 0x08020000
#define EEPROM_FIELD_CONFIG_START 0x08018000
#define EEPROM_PAGE_SIZE 0x00000400
#define EEPROM_FIELD_PAGES 2 // 16x16x8 = 2048 (rows x columns x bytes)
#define EEPROM_FIELD_SIZE 8
#define EEPROM_VARIABLE_SIZE 4
#define EEPROM_ADDRESS(start, number, size) ((start) + ((number) * (size))


#define TIM_UTIL_Handler htim1
#define TIM_UTIL_FREQ 10
#define TIM_UTIL_Instance TIM1

#define NR_OF_KEYS 16
#define NR_OF_SIGNALS 16
#define AVERAGE_BUFFER_SIZE 2


#define TACT_HYPERBOLA_MEASURE_PAR1 600.0f // y shift
#define TACT_HYPERBOLA_MEASURE_PAR2 2500000.0f // hyperbolic parameter
#define TACT_HYPERBOLA_MEASURE_PAR3 (-750.0f) // x shift

#define TACT_FORCE_DIVIDER 100.0f
#define TACT_IGNORANCE_LIMIT 3900 // 3800 na kwadracie

#define TACT_MEASURE(raw_analog) (uint16_t) (TACT_HYPERBOLA_MEASURE_PAR3 + \
                    (TACT_HYPERBOLA_MEASURE_PAR2 / ((raw_analog) - TACT_HYPERBOLA_MEASURE_PAR1)))


typedef struct {
    uint8_t  type;
    uint8_t  remained;
    uint8_t  key;
    uint8_t  channel;
    uint16_t angle;
    uint16_t distance;
} TactileField;


void Error_Handler();

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
