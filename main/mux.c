// read_3cd4051_repeat.c
#include <FreeRTOS.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <task.h>
#include "controles.h"


// mapear 24 canais (3 mux * 8). último pode ser '\0' se não usado.
// Ajuste conforme seus caracteres desejados.

/*
static const char button_map[3][8] = {
    { 'E','F','G','H','A','B','C','D', },   // MUX0 -> botões 0..7
    { 'I','J','K','L','M','N','O','P' },   // MUX1 -> botões 8..15
    { 'Q','R','S','T','U','V','W', '\n' }  // MUX2 -> botões 16..23 (último não usado)
};
*/
static const char button_map[3][8] = {
    { '8','F','G','H','A','B','C','!', },   // MUX0 -> botões 0..7
    { 'I','L','R','6','1','2','3','P' },   // MUX1 -> botões 8..15
    { 'Q','E','7','#','S','W','A', 'D' }  // MUX2 -> botões 16..23 (último não usado)
};

/*
# = Click
! = Esc

*/

// estruturas de estado por mux/canal
static bool last_raw[3][8];             // último valor cru lido (antes do debounce)
static uint32_t last_change_ms[3][8];   // when last_raw changed
static bool stable_state[3][8];         // estado estável após debounce (true = pressed)
static bool prev_stable[3][8];          // estado estável na iteração anterior (para detectar borda)
static uint32_t last_send_ms[3][8];     // último timestamp que enviou caractere (para repeat)


void mux_task(void *p) {
     // inicializa estados
    uint32_t t0 = now_ms();
    for (int m = 0; m < 3; m++) {
        for (int c = 0; c < 8; c++) {
            last_raw[m][c] = true;      // assumindo pull-up = HIGH quando solto
            last_change_ms[m][c] = t0;
            stable_state[m][c] = false;
            prev_stable[m][c] = false;
            last_send_ms[m][c] = 0;
        }
    }

    while (true) {
        uint32_t t = now_ms();

        // percorre canais 0..7 (mesma seleção para os 3 MUX)
        for (uint8_t ch = 0; ch < 8; ch++) {
            mux_select(ch);
            // leitura bruta em bloco
            uint32_t all = gpio_get_all();
            bool raw3[3];
            read_three_raw(all, raw3);

            for (int m = 0; m < 3; m++) {
                bool raw = raw3[m]; // 1 = solto, 0 = pressed
                bool raw_pressed = (raw == 0);

                // atualiza last_raw & last_change_ms
                if (raw_pressed != last_raw[m][ch]) {
                    last_raw[m][ch] = raw_pressed;
                    last_change_ms[m][ch] = t;
                } else {
                    // se igual e já passou debounce -> atualizar estado estável
                    if (!stable_state[m][ch] && (t - last_change_ms[m][ch] >= DEBOUNCE_MS)) {
                        stable_state[m][ch] = raw_pressed;
                    } else if (stable_state[m][ch] && (t - last_change_ms[m][ch] >= DEBOUNCE_MS)) {
                        // manter estado (se já era pressed e continua)
                        stable_state[m][ch] = raw_pressed;
                    }
                }

                // detectar borda solto -> pressionado
                if (!prev_stable[m][ch] && stable_state[m][ch]) {
                    // botão passou a PRESSED de forma estável
                    char out = button_map[m][ch];
                    if (out != '\0') {
                        uart_putc_raw(UART_ID, out);
                        // registra tempo do envio
                        last_send_ms[m][ch] = t + REPEAT_INITIAL_MS; // inicial delay antes do próximo repeat
                    }
                } else if (stable_state[m][ch]) {
                    // já estava pressionado: checar se é hora do próximo repeat
                    if ((int32_t)(t - last_send_ms[m][ch]) >= (int32_t)REPEAT_INTERVAL_MS) {
                        char out = button_map[m][ch];
                        if (out != '\0') {
                            uart_putc_raw(UART_ID, out);
                            last_send_ms[m][ch] = t;
                        }
                    }
                }
                prev_stable[m][ch] = stable_state[m][ch];
            }
        }

        // atraso curto entre varreduras para não saturar CPU
        vTaskDelay(pdMS_TO_TICKS(VARREDURA_DELAY_MS));
    }
}


int main() {
    stdio_init_all();
    init_pins_uart();
    printf("Init leitura 3x CD4051 com repeat enquanto pressionado\n");

    xTaskCreate(mux_task, "oled_task", 4095, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true)
    {
        /* code */
    }
    

    return 0;
}
