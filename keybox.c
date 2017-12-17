#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencmsis/core_cm3.h>
#include <stddef.h>
#include "debug.h"
#include "cie1931.h"

// Configuration:
//
// UART TX  = PB6
// UART RX  = PB7
//
// Left:
//
// LED R    = PA8
// LED G    = PA10
// LED B    = PA9
// Button   = PA15
// Solenoid = PA12
// Limit    = PB14
// Key      = PB15
//
// Right:
//
// LED R    = PA11
// LED G    = PB8
// LED B    = PB9
// Button   = PB5
// Solenoid = PB4
// Limit    = PB12
// Key      = PB13
//
// Serial packet format:
//
// [5a] [length of payload] [payload] [...] [checksum]
//
// Checksum is the 8-bit sum of the length and payload bytes.

#define DEBUG_LED_PORT GPIOC
#define DEBUG_LED_BIT GPIO13

#define LEFT_BUTTON_PORT GPIOA
#define LEFT_BUTTON_BIT GPIO15

#define LEFT_SOLENOID_PORT GPIOA
#define LEFT_SOLENOID_BIT GPIO12

#define LEFT_LIMIT_PORT GPIOB
#define LEFT_LIMIT_BIT GPIO14

#define LEFT_KEY_PORT GPIOB
#define LEFT_KEY_BIT GPIO15

#define RIGHT_BUTTON_PORT GPIOB
#define RIGHT_BUTTON_BIT GPIO5

#define RIGHT_SOLENOID_PORT GPIOB
#define RIGHT_SOLENOID_BIT GPIO4

#define RIGHT_LIMIT_PORT GPIOB
#define RIGHT_LIMIT_BIT GPIO12

#define RIGHT_KEY_PORT GPIOB
#define RIGHT_KEY_BIT GPIO13

static uint8_t ignore8 __attribute__((unused));
static uint16_t ignore16 __attribute__((unused));

static volatile uint32_t left_solenoid_on = 0;
static volatile uint32_t right_solenoid_on = 0;

static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_24mhz();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_TIM4);
}

static void gpio_setup_input(uint32_t gpioport, uint32_t gpios) {
    gpio_set_mode(gpioport, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, gpios);
    gpio_set(gpioport, gpios);
}

static void gpio_setup(void) {
    // Remap USART1 to PB6/PB6

    gpio_primary_remap(
        AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON,
        AFIO_MAPR_USART1_REMAP);

    // Debug LED

    gpio_set(DEBUG_LED_PORT, DEBUG_LED_BIT);
    gpio_set_mode(DEBUG_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, DEBUG_LED_BIT);

    // Left door

    gpio_setup_input(LEFT_BUTTON_PORT, LEFT_BUTTON_BIT);
    gpio_setup_input(LEFT_LIMIT_PORT, LEFT_LIMIT_BIT);
    gpio_setup_input(LEFT_KEY_PORT, LEFT_KEY_BIT);

    gpio_clear(LEFT_SOLENOID_PORT, LEFT_SOLENOID_BIT);
    gpio_set_mode(LEFT_SOLENOID_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
        LEFT_SOLENOID_BIT);

    // Right door

    gpio_setup_input(RIGHT_BUTTON_PORT, RIGHT_BUTTON_BIT);
    gpio_setup_input(RIGHT_LIMIT_PORT, RIGHT_LIMIT_BIT);
    gpio_setup_input(RIGHT_KEY_PORT, RIGHT_KEY_BIT);

    gpio_clear(RIGHT_SOLENOID_PORT, RIGHT_SOLENOID_BIT);
    gpio_set_mode(RIGHT_SOLENOID_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
        RIGHT_SOLENOID_BIT);

    // USART1

    gpio_set_mode(GPIO_BANK_USART1_RE_TX, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_USART1_RE_TX);

    gpio_set_mode(GPIO_BANK_USART1_RE_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
        GPIO_USART1_RE_RX);

    // LED PWM outputs

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_TIM1_CH1 | GPIO_TIM1_CH2 | GPIO_TIM1_CH3 | GPIO_TIM1_CH4);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_TIM4_CH3 | GPIO_TIM4_CH4);
}

static void timer_setup_config(uint32_t timer) {
    timer_reset(timer);
    timer_set_prescaler(timer, 0);
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(timer, 16384);
    timer_enable_preload(timer);
    timer_enable_counter(timer);
}

static void timer_setup_oc(uint32_t timer, uint32_t oc) {
    timer_set_oc_value(timer, oc, 0);
    timer_set_oc_mode(timer, oc, TIM_OCM_PWM1);
    timer_enable_oc_output(timer, oc);
}

static void timer_setup(void) {
    timer_setup_config(TIM1);
    timer_setup_oc(TIM1, TIM_OC1);
    timer_setup_oc(TIM1, TIM_OC2);
    timer_setup_oc(TIM1, TIM_OC3);
    timer_setup_oc(TIM1, TIM_OC4);
    timer_enable_break_main_output(TIM1);

    timer_setup_config(TIM4);
    timer_setup_oc(TIM4, TIM_OC3);
    timer_setup_oc(TIM4, TIM_OC4);
}

static void systick_setup(void) {
    // Tick every 1ms

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(24000 - 1);
    systick_interrupt_enable();
    systick_counter_enable();
}

static void usart_setup(void) {
    nvic_enable_irq(NVIC_USART1_IRQ);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    usart_enable_rx_interrupt(USART1);

    usart_enable(USART1);
}

static uint8_t get_status_byte(void) {
    uint8_t r = 0;

    if (!gpio_get(LEFT_BUTTON_PORT, LEFT_BUTTON_BIT))
        r |= 0x01;
    if (!gpio_get(LEFT_LIMIT_PORT, LEFT_LIMIT_BIT))
        r |= 0x02;
    if (!gpio_get(LEFT_KEY_PORT, LEFT_KEY_BIT))
        r |= 0x04;
    if (left_solenoid_on > 0)
        r |= 0x08;

    if (!gpio_get(RIGHT_BUTTON_PORT, RIGHT_BUTTON_BIT))
        r |= 0x10;
    if (!gpio_get(RIGHT_LIMIT_PORT, RIGHT_LIMIT_BIT))
        r |= 0x20;
    if (!gpio_get(RIGHT_KEY_PORT, RIGHT_KEY_BIT))
        r |= 0x40;
    if (right_solenoid_on > 0)
        r |= 0x80;

    return r;
}

static void process_command(uint8_t *buf, size_t len) {
    int cmd = buf[0];

    if (len == 4 && cmd == 0x01) { // Left LED
        timer_set_oc_value(TIM1, TIM_OC1, cie_table[buf[3]]);
        timer_set_oc_value(TIM1, TIM_OC2, cie_table[buf[1]]);
        timer_set_oc_value(TIM1, TIM_OC3, cie_table[buf[2]]);
    } else if (len == 4 && cmd == 0x11) { // Right LED
        timer_set_oc_value(TIM1, TIM_OC4, cie_table[buf[1]]);
        timer_set_oc_value(TIM4, TIM_OC3, cie_table[buf[3]]);
        timer_set_oc_value(TIM4, TIM_OC4, cie_table[buf[2]]);
    } else if (len == 3 && cmd == 0x02) { // Left solenoid
        left_solenoid_on = (buf[2] << 8) | buf[1];
        gpio_set(LEFT_SOLENOID_PORT, LEFT_SOLENOID_BIT);
    } else if (len == 3 && cmd == 0x12) { // Right solenoid
        right_solenoid_on = (buf[2] << 8) | buf[1];
        gpio_set(RIGHT_SOLENOID_PORT, RIGHT_SOLENOID_BIT);
    }
}

void usart1_isr(void) {
    enum { HEADER = 0, LENGTH = 1, PAYLOAD = 2, CHECKSUM = 3 };

    static uint32_t state = HEADER;
    static uint8_t checksum = 0;
    static size_t len = 0, i = 0;
    static uint8_t buf[40];

    if (USART_SR(USART1) & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t b = usart_recv(USART1);

        switch (state) {
            case HEADER:
                if (b == 0x5a)
                    state = LENGTH;

                break;

            case LENGTH:
                if (b <= sizeof(buf)/sizeof(buf[0])) {
                    len = b;
                    i = 0;
                    checksum = b;
                    state = PAYLOAD;
                } else {
                    state = HEADER;
                }

                break;

            case PAYLOAD:
                buf[i] = b;
                i++;
                checksum += b;

                if (i == len)
                    state = CHECKSUM;

                break;

            case CHECKSUM:
                if (b == checksum)
                    process_command(buf, len);

                state = HEADER;

                break;
        }
    }
}

void sys_tick_handler(void) {
    static uint32_t ticks = 0;

    if (left_solenoid_on > 0) {
        left_solenoid_on--;

        if (left_solenoid_on == 0)
            gpio_clear(LEFT_SOLENOID_PORT, LEFT_SOLENOID_BIT);
    }

    if (right_solenoid_on > 0) {
        right_solenoid_on--;

        if (right_solenoid_on == 0)
            gpio_clear(RIGHT_SOLENOID_PORT, RIGHT_SOLENOID_BIT);
    }

    // Report status every 10ms

    ticks++;
    if (ticks >= 10) {
        if (USART_SR(USART1) & USART_SR_TXE)
            usart_send(USART1, get_status_byte());

        ticks = 0;
    }
}

int main(void) {
    clock_setup();
    gpio_setup();
    systick_setup();
    usart_setup();
    timer_setup();
    debug_setup();

    printf("setup done\r\n");

    for (;;)
        __WFI();
}
