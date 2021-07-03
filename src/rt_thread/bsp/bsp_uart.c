#include "bsp_uart.h"
#include <rtdevice.h>
#include "uart.h"
#include "gpio.h"

struct hc32_uart {
	M0P_UART_TypeDef* uart_device;
	IRQn_Type irq;
};

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg) {
	struct stm32_uart* uart;
	stc_uart_config_t  stcConfig;
	stc_uart_irq_cb_t stcUartIrqCb;
    stc_uart_baud_t stcBaud;
	DDL_ZERO_STRUCT(stcBaud);



	DDL_ZERO_STRUCT(stcConfig);
    DDL_ZERO_STRUCT(stcUartIrqCb);

	RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
}

// static const struct rt_uart_ops hc32_uart_ops =
// {
//     hc32_configure,
//     hc32_control,
//     hc32_putc,
//     hc32_getc,
// };

void TxIntCallback(void) {

}
void RxIntCallback(void) {

}

static void GPIO_Configuration(void) {
	stc_gpio_config_t stcGpioCfg;
    DDL_ZERO_STRUCT(stcGpioCfg);
	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
#ifdef RT_USING_UART1
	stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortA,GpioPin2,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin2,GpioAf1);//TX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortA,GpioPin3,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin3,GpioAf1);//RX
#endif
#ifdef RT_USING_UART2

#endif
}

void rt_hw_usart_init(void)
{
    struct stm32_uart* uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    GPIO_Configuration();

#ifdef RT_USING_UART1
//    uart = &uart1;
//    config.baud_rate = BAUD_RATE_115200;

//    serial1.ops    = &stm32_uart_ops;
//    serial1.config = config;

//    NVIC_Configuration(&uart1);

//    /* register UART1 device */
//    rt_hw_serial_register(&serial1, "uart1",
//                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
//                          uart);
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART2
//    uart = &uart2;

//    config.baud_rate = BAUD_RATE_115200;
//    serial2.ops    = &stm32_uart_ops;
//    serial2.config = config;

//    NVIC_Configuration(&uart2);

//    /* register UART1 device */
//    rt_hw_serial_register(&serial2, "uart2",
//                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
//                          uart);
#endif /* RT_USING_UART2 */
}

