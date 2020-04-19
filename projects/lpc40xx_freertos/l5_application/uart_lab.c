#include "uart_lab.h"
#include "FreeRTOS.h"
#include "lpc40xx.h"
#include "lpc_peripherals.h"
#include "queue.h"
#include "task.h"
#include "uart_printf.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static QueueHandle_t your_uart_rx_queue;

typedef LPC_UART_TypeDef lpc_uart;

typedef struct {
  lpc_uart *registers;
  QueueHandle_t queue_transmit;
  QueueHandle_t queue_receive;
} uart_s;

static uart_s uart_map[] = {
    {(lpc_uart *)LPC_UART2},
    {(lpc_uart *)LPC_UART3},
};

static const lpc_peripheral_e uart_peripheral_number[] = {LPC_PERIPHERAL__UART2, LPC_PERIPHERAL__UART3};

void uart_lab__init(uart_number_e uart, uint32_t peripheral_clock, uint32_t baud_rate) {
  lpc_peripheral__turn_on_power_to(uart_peripheral_number[uart]);

  lpc_uart *UART_number = uart_map[uart].registers;

  const uint16_t divider = (uint16_t)((peripheral_clock / (16 * baud_rate)));

  UART_number->LCR = (1 << 7);

  UART_number->DLM = (divider >> 8) & 0xFF;
  UART_number->DLL = (divider >> 0) & 0xFF;
  UART_number->FDR = (1 << 4);

  const uint32_t default_reset_fdr_value = (1 << 4);
  UART_number->FDR = default_reset_fdr_value;
  const uint8_t eight_bit_datalen = 3;

  // 2-stop bits helps improve baud rate error; you can remove this if bandwidth is critical to you
  const uint8_t stop_bits_is_2 = (1 << 2);
  UART_number->LCR = eight_bit_datalen | stop_bits_is_2;
}

bool uart_lab__polled_get(uart_number_e uart, char *input_byte) {
  lpc_uart *UART_number = uart_map[uart].registers;
  while (!(UART_number->LSR & (1 << 0))) {
    //
  }
  *input_byte = UART_number->RBR;

  return true;
}

bool uart_lab__polled_put(uart_number_e uart, char output_byte) {
  lpc_uart *UART_number = uart_map[uart].registers;

  while (!(UART_number->LSR & (1 << 5))) {
    //
  }
  UART_number->THR = output_byte;
  while (!(UART_number->LSR & (1 << 5))) {
    //
  }

  return true;
}

static void your_receive_interrupt(void) {

  if (!(LPC_UART3->IIR & (1 << 0))) {
    while (LPC_UART3->LSR & (1 << 0)) {

      char byte = LPC_UART3->RBR;
      xQueueSendFromISR(your_uart_rx_queue, &byte, NULL);
    }
  }
}

void uart__enable_receive_interrupt(uart_number_e uart_number) {

  lpc_peripheral__enable_interrupt(LPC_PERIPHERAL__UART3, your_receive_interrupt);
  LPC_UART3->IER |= (1 << 0); // receive_data_available interrupt
  LPC_UART3->IER |= (1 << 1); // UnLSR
  LPC_UART3->IER |= (1 << 2); // Receive Line Status in
  your_uart_rx_queue = xQueueCreate(16, sizeof(char));
}

bool uart_lab__get_char_from_queue(char *input_byte, uint32_t timeout) {

  return xQueueReceive(your_uart_rx_queue, input_byte, timeout);
}
