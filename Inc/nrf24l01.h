#pragma once

#include <stdbool.h>
#include <stdint.h>

//#if defined STM32F0
//#include "stm32f0xx_hal.h"
//#elif defined STM32F1
#include "stm32f1xx_hal.h"
//#elif defined STM32F4
//#include "stm32f4xx_hal.h"
//#endif

/* Registers */
typedef enum {
    NRF_CONFIG      = 0x00,
    NRF_EN_AA       = 0x01,
    NRF_EN_RXADDR   = 0x02,
    NRF_SETUP_AW    = 0x03,
    NRF_SETUP_RETR  = 0x04,
    NRF_RF_CH       = 0x05,
    NRF_RF_SETUP    = 0x06,
    NRF_STATUS      = 0x07,
    NRF_OBSERVE_TX  = 0x08,
    NRF_CD          = 0x09,
    NRF_RX_ADDR_P0  = 0x0A,
    NRF_RX_ADDR_P1  = 0x0B,
    NRF_RX_ADDR_P2  = 0x0C,
    NRF_RX_ADDR_P3  = 0x0D,
    NRF_RX_ADDR_P4  = 0x0E,
    NRF_RX_ADDR_P5  = 0x0F,
    NRF_TX_ADDR     = 0x10,
    NRF_RX_PW_P0    = 0x11,
    NRF_RX_PW_P1    = 0x12,
    NRF_RX_PW_P2    = 0x13,
    NRF_RX_PW_P3    = 0x14,
    NRF_RX_PW_P4    = 0x15,
    NRF_RX_PW_P5    = 0x16,
    NRF_FIFO_STATUS = 0x17,
    NRF_DYNPD       = 0x1C,
    NRF_FEATURE     = 0x1D
} NRF_REGISTER;

/* Commands */
typedef enum {
    NRF_CMD_R_REGISTER         = 0x00,
    NRF_CMD_W_REGISTER         = 0x20,
    NRF_CMD_R_RX_PAYLOAD       = 0x61,
    NRF_CMD_W_TX_PAYLOAD       = 0xA0,
    NRF_CMD_FLUSH_TX           = 0xE1,
    NRF_CMD_FLUSH_RX           = 0xE2,
    NRF_CMD_REUSE_TX_PL        = 0xE3,
    NRF_CMD_ACTIVATE           = 0x50,
    NRF_CMD_R_RX_PL_WID        = 0x60,
    NRF_CMD_W_ACK_PAYLOAD      = 0xA8,
    NRF_CMD_W_TX_PAYLOAD_NOACK = 0xB0,
    NRF_CMD_NOP                = 0xFF
} NRF_COMMAND;

typedef enum {
    NRF_DATA_RATE_250KBPS = 1,
    NRF_DATA_RATE_1MBPS   = 0,
    NRF_DATA_RATE_2MBPS   = 2
} NRF_DATA_RATE;

typedef enum {
    NRF_TX_PWR_M18dBm = 0,
    NRF_TX_PWR_M12dBm = 1,
    NRF_TX_PWR_M6dBm  = 2,
    NRF_TX_PWR_0dBm   = 3
} NRF_TX_PWR;

typedef enum {
    NRF_ADDR_WIDTH_3 = 1,
    NRF_ADDR_WIDTH_4 = 2,
    NRF_ADDR_WIDTH_5 = 3
} NRF_ADDR_WIDTH;

typedef enum { NRF_CRC_WIDTH_1B = 0, NRF_CRC_WIDTH_2B = 1 } NRF_CRC_WIDTH;

typedef enum { NRF_STATE_RX = 1, NRF_STATE_TX = 0 } NRF_TXRX_STATE;

typedef enum { NRF_OK, NRF_ERROR, NRF_INVALID_ARGUMENT } NRF_RESULT;

typedef struct {

    NRF_DATA_RATE  data_rate;
    NRF_TX_PWR     tx_power;
    NRF_CRC_WIDTH  crc_width;
    NRF_ADDR_WIDTH addr_width;

    uint8_t        payload_length;
    uint8_t        retransmit_count;
    uint8_t        retransmit_delay;
    uint8_t        rf_channel;
    const uint8_t* rx_address;
    const uint8_t* tx_address;

    /* Must be sufficient size according to payload_length */
    uint8_t* rx_buffer;

    SPI_HandleTypeDef* spi;
    uint32_t           spi_timeout;

    GPIO_TypeDef* csn_port;
    uint16_t      csn_pin;

    GPIO_TypeDef* ce_port;
    uint16_t      ce_pin;

    GPIO_TypeDef* irq_port;
    uint16_t      irq_pin;

} nrf24l01_config;

typedef struct {
    nrf24l01_config config;

    volatile uint8_t        tx_busy;
    volatile NRF_RESULT     tx_result;
    volatile uint8_t        rx_busy;
    volatile NRF_TXRX_STATE state;

} nrf24l01;

/* Initialization routine */
NRF_RESULT nrf_init(nrf24l01* dev, nrf24l01_config* config);

/* EXTI Interrupt Handler
 *
 * You must call this function on Falling edge trigger detection interrupt
 * handler, typically, from HAL_GPIO_EXTI_Callback  */
void nrf_irq_handler(nrf24l01* dev);

/* Asynchronous Data Receiving (__weak)
 *
 * Override this function to handle received data asynchronously,
 * default implementation is used in favor of nrf_receive_packet for blocking
 * data receiving */
void nrf_packet_received_callback(nrf24l01* dev, uint8_t* data);

/* Blocking Data Receiving
 *
 * Blocks until the data has arrived, then returns a pointer to received data.
 * Please note, once nrf_packet_received_callback routine is overridden, this
 * one will stop working. */
const uint8_t* nrf_receive_packet(nrf24l01* dev);

/* Blocking Data Sending
 *
 * If the AA is enabled (default), this method will return:
 *   NRF_OK - the data has been acknowledged by other party
 *   NRF_ERROR - the data has not been received (maximum retransmissions has
 * occurred) If the AA is disabled, returns NRF_OK once the data has been
 * transmitted (with no guarantee the data was actually received). */
NRF_RESULT nrf_send_packet(nrf24l01* dev, const uint8_t* data);

/* Blocking Data Sending, with NO_ACK flag
 *
 * Disables the AA for this packet, thus this method always returns NRF_OK */
NRF_RESULT nrf_send_packet_noack(nrf24l01* dev, const uint8_t* data);

/* Non-Blocking Data Sending */
NRF_RESULT nrf_push_packet(nrf24l01* dev, const uint8_t* data);

/* LOW LEVEL STUFF (you don't have to look in here...)*/
NRF_RESULT nrf_send_command(nrf24l01* dev, NRF_COMMAND cmd, const uint8_t* tx,
                            uint8_t* rx, uint8_t len);
/* CMD */
NRF_RESULT nrf_read_register(nrf24l01* dev, uint8_t reg, uint8_t* data);
NRF_RESULT nrf_write_register(nrf24l01* dev, uint8_t reg, uint8_t* data);
NRF_RESULT nrf_read_rx_payload(nrf24l01* dev, uint8_t* data);
NRF_RESULT nrf_write_tx_payload(nrf24l01* dev, const uint8_t* data);
NRF_RESULT nrf_write_tx_payload_noack(nrf24l01* dev, const uint8_t* data);
NRF_RESULT nrf_flush_rx(nrf24l01* dev);
NRF_RESULT nrf_flush_tx(nrf24l01* dev);

/* RF_SETUP */
NRF_RESULT nrf_set_data_rate(nrf24l01* dev, NRF_DATA_RATE rate);
NRF_RESULT nrf_set_tx_power(nrf24l01* dev, NRF_TX_PWR pwr);
NRF_RESULT nrf_set_ccw(nrf24l01* dev, bool activate);

/* STATUS */
NRF_RESULT nrf_clear_interrupts(nrf24l01* dev);

/* RF_CH */
NRF_RESULT nrf_set_rf_channel(nrf24l01* dev, uint8_t ch);

/* SETUP_RETR */
NRF_RESULT nrf_set_retransmittion_count(nrf24l01* dev, uint8_t count);
NRF_RESULT nrf_set_retransmittion_delay(nrf24l01* dev, uint8_t delay);

/* SETUP_AW */
NRF_RESULT nrf_set_address_width(nrf24l01* dev, NRF_ADDR_WIDTH width);

/* EN_RXADDR */
NRF_RESULT nrf_set_rx_pipes(nrf24l01* dev, uint8_t pipes);

/* EN_AA */
NRF_RESULT nrf_enable_auto_ack(nrf24l01* dev, uint8_t pipe);
// TODO disable AA?

/* CONFIG */
NRF_RESULT nrf_enable_crc(nrf24l01* dev, bool activate);
NRF_RESULT nrf_set_crc_width(nrf24l01* dev, NRF_CRC_WIDTH width);
NRF_RESULT nrf_power_up(nrf24l01* dev, bool power_up);
NRF_RESULT nrf_rx_tx_control(nrf24l01* dev, NRF_TXRX_STATE rx);
NRF_RESULT nrf_enable_rx_data_ready_irq(nrf24l01* dev, bool activate);
NRF_RESULT nrf_enable_tx_data_sent_irq(nrf24l01* dev, bool activate);
NRF_RESULT nrf_enable_max_retransmit_irq(nrf24l01* dev, bool activate);

/* RX_ADDR_P0 */
NRF_RESULT nrf_set_rx_address_p0(nrf24l01*      dev,
                                 const uint8_t* address); // 5bytes of address

/* RX_ADDR_P1 */
NRF_RESULT nrf_set_rx_address_p1(nrf24l01*      dev,
                                 const uint8_t* address); // 5bytes of address

/* TX_ADDR */
NRF_RESULT nrf_set_tx_address(nrf24l01*      dev,
                              const uint8_t* address); // 5bytes of address

/* RX_PW_P0 */
NRF_RESULT nrf_set_rx_payload_width_p0(nrf24l01* dev, uint8_t width);

/* RX_PW_P1 */
NRF_RESULT nrf_set_rx_payload_width_p1(nrf24l01* dev, uint8_t width);
