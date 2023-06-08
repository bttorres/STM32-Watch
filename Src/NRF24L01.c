/*
 * NRF24L01.c
 *
 *  Created on: Feb 1, 2023
 *      Author: Brandon Torres
 */

#include "stm32g4xx_hal.h"
#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI	&hspi1

#define CE_PORT		GPIOF
#define CE_PIN		GPIO_PIN_0
#define CSN_PORT	GPIOB
#define CSN_PIN		GPIO_PIN_6

void CSN_select(void) {
	HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, GPIO_PIN_RESET);
}

void CSN_unselect(void) {
	HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, GPIO_PIN_SET);
}

void CE_enable(void) {
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);
}

void CE_disable(void) {
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_RESET);
}

// Write a single byte to the particular register
void nrf24_write_reg(uint8_t reg, uint8_t data) {
	uint8_t buf[2];
	buf[0] = reg | 1 << 5;
	buf[1] = data;

	// Pull the CSN pin LOW to select the device
	CSN_select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);

	// Pull the CSN pin HIGH to release the device
	CSN_unselect();
}

// Write multiple bytes starting from a particular register
void nrf24_write_reg_multi(uint8_t reg, uint8_t *data, int size) {
	uint8_t buf[2];
	buf[0] = reg | 1 << 5;
	// buf[1] = data;

	// Pull the CSN pin LOW to select the device
	CSN_select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);
	HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);

	// Pull the CSN pin HIGH to release the device
	CSN_unselect();
}

// Read a single byte from a particular register
uint8_t nrf24_read_reg(uint8_t reg) {
	uint8_t data = 0;
	// Pull the CSN pin LOW to select the device
	CSN_select();

	HAL_SPI_Transmit(NRF24_SPI, &reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 100);

	// Pull the CSN pin HIGH to release the device
	CSN_unselect();

	return data;
}

// Read multiple bytes from a particular register
void nrf24_read_reg_multi(uint8_t reg, uint8_t *data, int size) {
	// Pull the CSN pin LOW to select the device
	CSN_select();

	HAL_SPI_Transmit(NRF24_SPI, &reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

	// Pull the CSN pin HIGH to release the device
	CSN_unselect();
}

// send the command to the NRF
void nrf24_send_cmd(uint8_t cmd) {
	// Pull the CSN pin LOW to select the device
	CSN_select();

	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	// Pull the CSN pin HIGH to release the device
	CSN_unselect();
}

void nrf24_init(void) {
	// Disable the chip before configuring the device
	CE_disable();

	nrf24_write_reg(CONFIG, 0);			// Will be configured later
	nrf24_write_reg(EN_AA, 0);			// No auto ACK
	nrf24_write_reg(EN_RXADDR, 0);		// Not enabling any data pipe right now
	nrf24_write_reg(SETUP_AW, 0x03);	// 5 Bytes for the TX/RX address
	nrf24_write_reg(SETUP_RETR, 0);		// No retransmission
	nrf24_write_reg(RF_CH, 0);			// Will be set up during TX or RX
	nrf24_write_reg(RF_SETUP, 0x0E);	// Power = 0db, data rate = 2Mbps

	// Enable the chip after configuring the device
	CE_enable();
}

// Set up the TX mode
void nrf24_tx_mode(uint8_t *addr, uint16_t channel) {
	// Disable the chip before configuring the device
	CE_disable();

	nrf24_write_reg(RF_CH, channel);			// Select the channel
	nrf24_write_reg_multi(TX_ADDR, addr, 5);	// Write the TX address

	// Power up the device
	uint8_t config = nrf24_read_reg(CONFIG);
	config = config | (1 << 1);
	nrf24_write_reg(CONFIG, config);

	// Enable the chip after configuring the device
	CE_enable();
}

// Transmit the data
uint8_t nrf24_transmit(uint8_t *data) {
	uint8_t cmd_to_send = 0;

	// Pull the CSN pin LOW to select the device
	CSN_select();

	// Payload command
	cmd_to_send = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmd_to_send, 1, 100);

	// Send the payload
	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

	// Pull the CSN pin HIGH to release the device
	CSN_unselect();

	HAL_Delay(1);

	// Check the 4th bit of FIFO_STATUS to know if the TX fifo is empty
	uint8_t fifo_status = nrf24_read_reg(FIFO_STATUS);
	if ((fifo_status & (1 << 4)) && (!(fifo_status & (1 << 3)))) {
		cmd_to_send = FLUSH_TX;
		nrf24_send_cmd(cmd_to_send);
		return 1;
	}
	return 0;
}

// Set up the RX mode
void nrf24_rx_mode(uint8_t *addr, uint8_t channel) {
	// Disable the chip before configuring the device
	CE_disable();

	nrf24_write_reg(RF_CH, channel);			// Select the channel

	// Select data pipe 1
	uint8_t en_rxaddr = nrf24_read_reg(EN_RXADDR);
	en_rxaddr = en_rxaddr | (1 << 1);
	nrf24_write_reg(EN_RXADDR, en_rxaddr);

	nrf24_write_reg_multi(RX_ADDR_P1, addr, 5);	// Write the RX address
	nrf24_write_reg(RX_PW_P1, 32);			// 32 byte payload size for pipe 1

	// Power up the device in RX mode
	uint8_t config = nrf24_read_reg(CONFIG);
	config = config | (1 << 1) | (1 << 0);
	nrf24_write_reg(CONFIG, config);

	// Enable the chip after configuring the device
	CE_enable();
}

uint8_t is_data_available(int pipe_num) {
	uint8_t status = nrf24_read_reg(STATUS);

	if ((status & (1 << 6)) && (status & (pipe_num << 1))) {
		nrf24_write_reg(STATUS, (1 << 6));
		return 1;
	}
	return 0;
}

void nrf24_receive(uint8_t *data) {
	uint8_t cmd_to_send = 0;

	// Pull the CSN pin LOW to select the device
	CSN_select();

	// Payload command
	cmd_to_send = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmd_to_send, 1, 100);

	// Send the payload
	HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

	// Pull the CSN pin HIGH to release the device
	CSN_unselect();

	HAL_Delay(1);

	cmd_to_send = FLUSH_RX;
	nrf24_send_cmd(cmd_to_send);
}
