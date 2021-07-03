#ifndef _SYS_PORT_H
#define _SYS_PORT_H

#include "gpio.h"
#include "board_stkhc32l13x.h"

/* system LED */
#define SYS_LED_PORT       GpioPortD
#define SYS_LED_PIN        GpioPin5
#define SYS_LED_ON				 Gpio_SetIO(SYS_LED_PORT, SYS_LED_PIN)
#define SYS_LED_OFF				 Gpio_ClrIO(SYS_LED_PORT, SYS_LED_PIN)
#define SYS_LED_OUT_STA		 Gpio_ReadOutputIO(SYS_LED_PORT,SYS_LED_PIN)

/* Test LED */
#define Red_LED_PORT       GpioPortA
#define Red_LED_PIN        GpioPin3
#define Red_LED_ON				 Gpio_SetIO(Red_LED_PORT, Red_LED_PIN)
#define Red_LED_OFF				 Gpio_ClrIO(Red_LED_PORT, Red_LED_PIN)
#define Red_LED_OUT_STA		 Gpio_ReadOutputIO(Red_LED_PORT,Red_LED_PIN)

/* UART0 */
#define UART0_TX_PORT			 GpioPortA
#define UART0_TX_PIN			 GpioPin8
#define UART0_RX_PORT			 GpioPortA
#define UART0_RX_PIN			 GpioPin10

/* XTL output */
#define SYS_XTL_PORT       GpioPortA
#define SYS_XTL_PIN        GpioPin5

#endif
