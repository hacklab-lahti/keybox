#include "debug.h"

#ifdef DEBUG_UART

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#define PASTE_(a, b) a ## b
#define PASTE(a, b) PASTE_(a, b)

#ifndef DEBUG_UART_PIN
#   define DEBUG_UART_PIN DEBUG_UART
#endif

#ifndef DEBUG_UART_BAUDRATE
#   define DEBUG_UART_BAUDRATE 115200
#endif

#define USART_DEBUG PASTE(USART, DEBUG_UART)

void debug_setup(void) {
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(PASTE(RCC_USART, DEBUG_UART));

#if DEBUG_UART == 1
    rcc_periph_clock_enable(RCC_GPIOA);
#elif DEBUG_UART == 2
    rcc_periph_clock_enable(RCC_GPIOA);
#elif DEBUG_UART == 3
    rcc_periph_clock_enable(RCC_GPIOB);
#endif

    gpio_set_mode(
        PASTE(PASTE(GPIO_BANK_USART, DEBUG_UART_PIN), _TX),
        GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        PASTE(PASTE(GPIO_USART, DEBUG_UART_PIN), _TX));

    usart_set_baudrate(USART_DEBUG, 115200);
    usart_set_databits(USART_DEBUG, 8);
    usart_set_stopbits(USART_DEBUG, USART_STOPBITS_1);
    usart_set_parity(USART_DEBUG, USART_PARITY_NONE);
    usart_set_flow_control(USART_DEBUG, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART_DEBUG, USART_MODE_TX);

    usart_enable(USART_DEBUG);
}

int _write(int file, char *ptr, int len);

int _write(int file, char *ptr, int len) {
    (void)file;

    for (int i = 0; i < len; i++)
        usart_send_blocking(USART_DEBUG, ptr[i]);

    return len;
}

#else

void debug_setup(void) { }

#endif
