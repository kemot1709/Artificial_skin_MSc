#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


#define UART_IN_BUFF_SIZE           260 // 1+1+256+2 (komenda, ilość danych, dane, crc)
#define UART_TIMEOUT                100 // ms


#define USB_MSG_FUNCTION_BYTE 0x00
#define USB_MSG_LENGTH_BYTE 0x01
#define USB_MSG_DATA_FIRST_BYTE 0x02
#define USB_MSG_CRC_FIRST_BYTE(data_length) (USB_MSG_DATA_FIRST_BYTE + (data_length))


#define USB_START_BYTE                   0x02


#define USB_FUNCTION_COMMAND_REQUEST 0x11
#define USB_FUNCTION_COMMAND_ANSWER 0x19
#define USB_FUNCTION_REGISTER_ONE_WRITE 0x21
#define USB_FUNCTION_REGISTER_ONE_READ 0x22
#define USB_FUNCTION_REGISTER_MULTIPLE_WRITE 0x25
#define USB_FUNCTION_REGISTER_MULTIPLE_READ 0x26
#define USB_FUNCTION_REGISTER_ANSWER 0x29
#define USB_FUNCTION_CONFIG_WRITE 0x35
#define USB_FUNCTION_CONFIG_READ 0x36
#define USB_FUNCTION_CONFIG_ANSWER 0x39


#define USB_COMMAND_ERASE_ALL_FIELDS


#define USB_REGISTER_BAUDRATE
#define USB_REGISTER_ADC_SAMPLING_TIME


#define FIELD_TYPE_TACTILE_SIDE 0x11
#define FIELD_TYPE_TACTILE_TABLE 0x12
#define FIELD_TYPE_TEMPERATURE 0x21


#define USB_UART_TypeDef            huart3
#define USB_UART_Instance           USART3
#define USB_UART_IRQ                USART3_IRQn
#define USB_UART_BaudRate           115200
#define USB_UART_CLK_ENABLE         __HAL_RCC_USART3_CLK_ENABLE()
#define USB_UART_GPIO_CLK_ENABLE    __HAL_RCC_GPIOC_CLK_ENABLE()
#define USB_UART_TX_Pin             GPIO_PIN_10
#define USB_UART_RX_Pin             GPIO_PIN_11
#define USB_UART_Port               GPIOC
#define USB_UART_REMAP


void usb_uart_Init();

void usb_send_string(const char *msg);

void usb_reset_receive();

void uart_check_timeout();

uint8_t message_interpret(const uint8_t *buffer, uint8_t length);

uint16_t get_CRC(uint8_t *nData, uint8_t wLength);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

