#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void sleep(uint32_t ticks) {
    while (ticks--)
        __asm__("nop");
}

int main(void) {
    // Blink to test
    
    rcc_periph_clock_enable(RCC_GPIOC);

    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    while (1) {
        gpio_clear(GPIOC, GPIO13);
        sleep(800000);

        gpio_set(GPIOC, GPIO13);
        sleep(800000);
    }
}
