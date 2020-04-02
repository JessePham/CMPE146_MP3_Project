#include "uart_lab.h"

static QueueHandle_t your_uart_rx_queue;
static const LPC_UART_TypeDef *uart_memory_map[] = {LPC_UART2, LPC_UART3};

void uart_lab__init(uart_number_e uart, uint32_t peripheral_clock, uint32_t baud_rate) {
  // Refer to LPC User manual and setup the register bits correctly
  // The first page of the UART chapter has good instructions
  // a) Power on Peripheral
  // b) Setup DLL, DLM, FDR, LCR registers
  uint16_t divider = (peripheral_clock / (16 * baud_rate));

  if (uart == UART_2) {
    LPC_SC->PCONP |= (1 << 24);
  } else if (uart == UART_3) {
    LPC_SC->PCONP |= (1 << 25);
  }

  LPC_UART3->LCR |= (1 << 7);
  ((LPC_UART_TypeDef *)uart_memory_map[uart])->LCR |= (1 << 7); // DLAB Enable
  ((LPC_UART_TypeDef *)uart_memory_map[uart])->DLL |= (divider >> 0);
  ((LPC_UART_TypeDef *)uart_memory_map[uart])->DLM |= (divider >> 8);
  ((LPC_UART_TypeDef *)uart_memory_map[uart])->LCR &= ~(1 << 7); // DLAB Disable

  ((LPC_UART_TypeDef *)uart_memory_map[uart])->LCR |= (3 << 0); // Sets to 8-bit characters
  ((LPC_UART_TypeDef *)uart_memory_map[uart])->FCR |= (1 << 0); // Enables FIFO
}

bool uart_lab__polled_get(uart_number_e uart, char *input_byte) {
  // a) Check LSR for Receive Data Ready
  while (!(uart_memory_map[uart]->LSR & (1 << 0))) {
    ;
  }
  // b) Copy data from RBR register to input_byte
  *input_byte = uart_memory_map[uart]->RBR;

  return true;
}

bool uart_lab__polled_put(uart_number_e uart, char output_byte) {
  // a) Check LSR for Transmit Hold Register Empty
  while (!(uart_memory_map[uart]->LSR & (1 << 5))) {
    ;
  }
  // b) Copy output_byte to THR register
  ((LPC_UART_TypeDef *)uart_memory_map[uart])->THR = output_byte;

  return true;
}

// Private function of our uart_lab.c
static void your_receive_interrupt(void) {
  // TODO: Read the IIR register to figure out why you got interrupted (pg.499)
  if ((LPC_UART3->IIR & (7 << 1)) == 0x6) {
    // Interrupted by Receive Line Status (RLS)
    fprintf(stderr, "\nRLS\n");
  } else if ((LPC_UART3->IIR & (7 << 1)) == 0x4) {
    // Interrupted by Receive Data Available (RDA)
    fprintf(stderr, "\nRDA\n");
  } else if ((LPC_UART3->IIR & (7 << 1)) == 0xc) {
    // Interrupted by Character Time-out Indicator (CTI)
    fprintf(stderr, "\nCTI\n");
  } else if ((LPC_UART3->IIR & (7 << 1)) == 0x2) {
    // Interrupted by THRE Interrupt
    fprintf(stderr, "\nTHRE INTERRUPT\n");
  }

  // TODO: Based on IIR status, read the LSR register to confirm if there is data to be read
  // TODO: Based on LSR status, read the RBR register and input the data to the RX Queue
  if (!(LPC_UART3->IIR & (1 << 0))) {
    while (!(LPC_UART3->LSR & (1 << 0))) {
      ;
    }
    const char byte = LPC_UART3->RBR;

    xQueueSendFromISR(your_uart_rx_queue, &byte, NULL);
  }
}

// Public function to enable UART interrupt
// TODO Declare this at the header file
void uart__enable_receive_interrupt(uart_number_e uart_number) {
  // TODO: Use lpc_peripherals.h to attach your interrupt

  if (uart_number == UART_3) {
    lpc_peripheral__enable_interrupt(LPC_PERIPHERAL__UART3, your_receive_interrupt);
  } else if (uart_number == UART_2) {
    lpc_peripheral__enable_interrupt(LPC_PERIPHERAL__UART2, your_receive_interrupt);
  }

  // TODO: Enable UART receive interrupt by reading the LPC User manual
  // Hint: Read about the IER register  (p. 498)
  LPC_UART3->IER |= (1 << 0);
  LPC_UART3->IER |= (1 << 1);
  LPC_UART3->IER |= (1 << 2);

  // TODO: Create your RX queue
  your_uart_rx_queue = xQueueCreate(10, sizeof(char)); // might just need sizeof(char)
}

// Public function to get a char from the queue (this function should work without modification)
// TODO: Declare this at the header file
bool uart_lab__get_char_from_queue(char *input_byte, uint32_t timeout) {
  return xQueueReceive(your_uart_rx_queue, input_byte, timeout);
}