#ifndef __BSP_UART
#define __BSP_UART
#include "ddl.h"
#include <rthw.h>
#include <rtthread.h>

#define RT_USING_UART1
#define RT_USING_UART2
#define UART_ENABLE_IRQ(n)            NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n)           NVIC_DisableIRQ((n))

void rt_hw_usart_init(void);

#endif
