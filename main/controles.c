#include "controles.h"

uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

void init_pins_uart(void) {
    // seletores
    for (int i = 0; i < 3; ++i) {
        gpio_init(sel_pins[i]);
        gpio_set_dir(sel_pins[i], GPIO_OUT);
        gpio_put(sel_pins[i], 0);
    }
    // SIGs
    for (int i = 0; i < 3; ++i) {
        gpio_init(sig_pins[i]);
        gpio_set_dir(sig_pins[i], GPIO_IN);
        gpio_pull_up(sig_pins[i]); // 1 = solto, 0 = pressionado
    }

    // inicializa UART
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

void mux_select(uint8_t ch) {
    for (int i = 0; i < 3; ++i) gpio_put(sel_pins[i], (ch >> i) & 0x1);
    sleep_us(MUX_DELAY_US);
}

// lê os 3 sinais de uma vez usando gpio_get_all()
void read_three_raw(uint32_t all, bool out_states[3]) {
    for (int i = 0; i < 3; ++i) {
        out_states[i] = ((all >> sig_pins[i]) & 1u) != 0;
    }
}