

#include "FreeRTOS.h"
#include "probe_config.h"


typedef struct{
    void (*setup)();
    void (*init)(uint baudrate);
    void (*deinit)();
    bool (*is_readable)();
    char (*getc)();
    size_t (*write)(const uint8_t* data, size_t len);
    void (*write_blocking)(const uint8_t* data, size_t len);
    void (*set_format)(uint data_bits, uint stop_bits, uart_parity_t parity);
    void (*set_break)(bool enable);

    TickType_t interval;
    uint cdc_tx_oe;
    bool was_connected;
    bool dtr;
    bool rts;

} UartInterface;



void uart_iface_uart0_setup(){
    gpio_set_function(PROBE_UART_TX, UART_FUNCSEL_NUM(uart0, PROBE_UART_TX));
    gpio_set_function(PROBE_UART_RX, UART_FUNCSEL_NUM(uart0, PROBE_UART_RX));
    gpio_set_pulls(PROBE_UART_TX, 1, 0);
    gpio_set_pulls(PROBE_UART_RX, 1, 0);
}
void uart_iface_uart0_init(uint baudrate){
    uart_init(PROBE_UART_INTERFACE, baudrate);
}
void uart_iface_uart0_deinit(){
    uart_deinit(PROBE_UART_INTERFACE);
}
bool uart_iface_uart0_is_readable(){
    return uart_is_readable(PROBE_UART_INTERFACE);
}
char uart_iface_uart0_getc(){
    return uart_getc(PROBE_UART_INTERFACE);
}
size_t uart_iface_uart0_write(const uint8_t* data, size_t len){
    uart_hw_t* hw = uart_get_hw(PROBE_UART_INTERFACE);
    for (size_t i = 0; i < len; ++i) {
        if (!uart_is_writable(PROBE_UART_INTERFACE)){
            return i;
        }
        hw->dr = *data++;
    }
    return len;
}
void uart_iface_uart0_write_blocking(const uint8_t* data, size_t len){
    uart_write_blocking(PROBE_UART_INTERFACE, data, len);
}
void uart_iface_uart0_set_format(uint data_bits, uint stop_bits, uart_parity_t parity){
    uart_set_format(PROBE_UART_INTERFACE, data_bits, stop_bits, parity);
}
void uart_iface_uart0_set_break(bool enable){
    uart_set_break(PROBE_UART_INTERFACE, enable);
}
const UartInterface UART_INTERFACE_UART0 = {
    uart_iface_uart0_setup,
    uart_iface_uart0_init,
    uart_iface_uart0_deinit,
    uart_iface_uart0_is_readable,
    uart_iface_uart0_getc,
    uart_iface_uart0_write,
    uart_iface_uart0_write_blocking,
    uart_iface_uart0_set_format,
    uart_iface_uart0_set_break,
    100,
    0,
    false,
    false,
    false,
};

#ifdef PROBE_USE_PIO_FOR_UART1

#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

PIO pio_tx_pio;
uint pio_tx_sm;
uint pio_tx_offset;

PIO pio_rx_pio;
uint pio_rx_sm;
uint pio_rx_offset;

void uart_iface_pio_setup(){
    {
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(
            &uart_tx_program,
            &pio_tx_pio, &pio_tx_sm, &pio_tx_offset,
            PROBE_UART1_TX, 1, true
        );
        hard_assert(success);
    }
    {
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(
            &uart_rx_program,
            &pio_rx_pio, &pio_rx_sm, &pio_rx_offset,
            PROBE_UART1_RX, 1, true
        );
        hard_assert(success);
    }
}
void uart_iface_pio_init(uint baudrate){
#if 1
    uart_tx_program_init(pio_tx_pio, pio_tx_sm, pio_tx_offset, PROBE_UART1_TX, baudrate);
    uart_rx_program_init(pio_rx_pio, pio_rx_sm, pio_rx_offset, PROBE_UART1_RX, baudrate);
#endif
}
void uart_iface_pio_deinit(){
}
bool uart_iface_pio_is_readable(){
#if 1
    return uart_rx_program_has_data(pio_rx_pio, pio_rx_sm);
#else
    return false;
#endif
}
char uart_iface_pio_getc(){
#if 1
    return uart_rx_program_getc(pio_rx_pio, pio_rx_sm);
#else
    return 0;
#endif
}
size_t uart_iface_pio_write(const uint8_t* data, size_t len){
}
void uart_iface_pio_write_blocking(const uint8_t* data, size_t len){
#if 1
    for (size_t c = 0; c < len; c++){
        uart_tx_program_putc(pio_tx_pio, pio_tx_sm, data[c]);
    }
#endif
}
void uart_iface_pio_set_format(uint data_bits, uint stop_bits, uart_parity_t parity){
    //  Not supported
}
void uart_iface_pio_set_break(bool enable){
}
const UartInterface UART_INTERFACE_PIO = {
    uart_iface_pio_setup,
    uart_iface_pio_init,
    uart_iface_pio_deinit,
    uart_iface_pio_is_readable,
    uart_iface_pio_getc,
    uart_iface_pio_write,
    uart_iface_pio_write_blocking,
    uart_iface_pio_set_format,
    uart_iface_pio_set_break,
    100,
    0,
    false,
    false,
    false,
};

#else

void uart_iface_uart1_setup(){
    gpio_set_function(PROBE_UART1_TX, UART_FUNCSEL_NUM(uart1, PROBE_UART1_TX));
    gpio_set_function(PROBE_UART1_RX, UART_FUNCSEL_NUM(uart1, PROBE_UART1_RX));
    gpio_set_pulls(PROBE_UART1_TX, 1, 0);
    gpio_set_pulls(PROBE_UART1_RX, 1, 0);
}
void uart_iface_uart1_init(uint baudrate){
    uart_init(PROBE_UART1_INTERFACE, baudrate);
}
void uart_iface_uart1_deinit(){
    uart_deinit(PROBE_UART1_INTERFACE);
}
bool uart_iface_uart1_is_readable(){
    return uart_is_readable(PROBE_UART1_INTERFACE);
}
char uart_iface_uart1_getc(){
    return uart_getc(PROBE_UART1_INTERFACE);
}
size_t uart_iface_uart1_write(const uint8_t* data, size_t len){
    uart_hw_t* hw = uart_get_hw(PROBE_UART1_INTERFACE);
    for (size_t i = 0; i < len; ++i) {
        if (!uart_is_writable(PROBE_UART1_INTERFACE)){
            return i;
        }
        hw->dr = *data++;
    }
    return len;
}
void uart_iface_uart1_write_blocking(const uint8_t* data, size_t len){
    uart_write_blocking(PROBE_UART1_INTERFACE, data, len);
}
void uart_iface_uart1_set_format(uint data_bits, uint stop_bits, uart_parity_t parity){
    uart_set_format(PROBE_UART1_INTERFACE, data_bits, stop_bits, parity);
}
void uart_iface_uart1_set_break(bool enable){
    uart_set_break(PROBE_UART1_INTERFACE, enable);
}
const UartInterface UART_INTERFACE_UART1 = {
    uart_iface_uart1_setup,
    uart_iface_uart1_init,
    uart_iface_uart1_deinit,
    uart_iface_uart1_is_readable,
    uart_iface_uart1_getc,
    uart_iface_uart1_write,
    uart_iface_uart1_write_blocking,
    uart_iface_uart1_set_format,
    uart_iface_uart1_set_break,
    100,
    0,
    false,
    false,
    false,
};



#endif

UartInterface UART_INTERFACES[] = {
  UART_INTERFACE_UART0,
#ifdef PROBE_USE_PIO_FOR_UART1
  UART_INTERFACE_PIO,
#else
  UART_INTERFACE_UART1,
#endif
};

