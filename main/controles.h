#ifndef CONTROLES_H
#define CONTROLES_H

// read_3cd4051_repeat.c
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

// Entrada de Dados de Todos os MUXes
#define S0 18
#define S1 17
#define S2 16
// SIGs (COM OUT) para os 3 MUXes
#define SIG0 11
#define SIG1 13
#define SIG2 12


// UART
#define UART_ID uart0
#define UART_BAUD 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Timing (ajuste à vontade)
#define MUX_DELAY_US 80          // tempo para o MUX estabilizar após selecionar canal
#define DEBOUNCE_MS 20           // tempo para considerar a leitura estável
#define REPEAT_INITIAL_MS 0      // atraso antes do primeiro repeat (0 = envia imediato e depois REPEAT_INTERVAL_MS)
#define REPEAT_INTERVAL_MS 120   // intervalo entre envios repetidos enquanto pressionado
#define VARREDURA_DELAY_MS 2          // atraso entre varreduras completas (ajusta latência / CPU)

// pinos arrays
static const uint sel_pins[3] = {S0, S1, S2};
static const uint sig_pins[3] = {SIG0, SIG1, SIG2};




 uint32_t now_ms();
 
 void mux_select(uint8_t ch);

 void read_three_raw(uint32_t all, bool out_states[3]);

void init_pins_uart();

#endif