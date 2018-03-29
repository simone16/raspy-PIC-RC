/* 
 *  Code for the PIC-nRF24L01+ communication.
 *
 *  Compile with: xc8 --chip=16f876A main.c
 *  Load with: #pk2cmd -P PIC16f876A [-X] -M -F main.hex
 *
 *  Author: Simone Pilon <wertyseek@gmail.com>
 */
// vim plugin header: ------------------
// Set compiler:
//!compiler=xc8
// Set device name:
//!device=16F876A
// -------------------------------------

#include <stdio.h>
#include <stdlib.h>

// CONFIGURATION WORD
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)
#pragma config DEBUG = OFF      // RB6 and 7 are general IO

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 20000000      // Osc. freq. in Hz

#include <xc.h>						// Macros and registers definitions

/*
 * Electrical connections:
 * 	RC0 -> CSN
 * 	RC5 -> MOSI
 * 	RC4 <- MISO
 * 	RC3 -> SCK
 * 	RB0 <- IRQ
 * 	RB1 -> CE
 *
 * 	RB2 -> test clock
 * 	RB3 -> test status
 *
 * Comments:
 *
 * nRF24L01+ usage:
 * 	SPI communication is handled by the SPI interrupt routine.
 * 	1. Load the bytes to be transmitted in spi_buffer (spi_buffer[0] 
 * 		is transmitted last, exclude the first command byte)
 * 		and set spi_counter to the number of bytes to transmit.
 * 	2. Load the command byte (the first) in SSPBUF register 
 * 	3. The rest of the communication is handled automatically, avoid writing
 * 		to spi_counter until spi_ready is 1 (communication is complete).
 */

// Project defines:
// Instruction Mnemonics (nRF24L01+)
#define RF_R_REGISTER 0x00
#define RF_W_REGISTER 0x20
#define RF_R_RX_PAYLOAD 0x61
#define RF_W_TX_PAYLOAD 0xA0
#define RF_FLUSH_TX 0xE1
#define RF_FLUSH_RX 0xE2
#define RF_REUSE_TX_PL 0xE3
#define RF_R_RX_PL_WID 0x60
#define RF_W_ACK_PAYLOAD 0xA8
#define RF_W_TX_PAYLOAD_NOACK 0xB0
#define RF_NOP 0xFF

// Register addresses (nRF24L01+)
#define RF_CONFIG 0x00
#define RF_EN_AA 0x01
#define RF_EN_RXADDR 0x02
#define RF_SETUP_AW 0x03
#define RF_SETUP_RETR 0x04
#define RF_RF_CH 0x05
#define RF_RF_SETUP 0x06
#define RF_STATUS 0x07
#define RF_OBSERVE_TX 0x08
#define RF_RPD 0x09
#define RF_RX_ADDR_P0 0x0A
#define RF_RX_ADDR_P1 0x0B
#define RF_RX_ADDR_P2 0x0C
#define RF_RX_ADDR_P3 0x0D
#define RF_RX_ADDR_P4 0x0E
#define RF_RX_ADDR_P5 0x0F
#define RF_TX_ADDR 0x10
#define RF_RX_PW_P0 0x11
#define RF_RX_PW_P1 0x12
#define RF_RX_PW_P2 0x13
#define RF_RX_PW_P3 0x14
#define RF_RX_PW_P4 0x15
#define RF_RX_PW_P5 0x16
#define RF_FIFO_STATUS 0x17
#define RF_DYNPD 0x1C
#define RF_FEATURE 0x1D

unsigned char spi_buffer[33];		// Bytes queued for transmission
unsigned char spi_counter = 0;	// Number of bytes to transmit
unsigned char spi_ready = 1;		// SPI ready flag

// Main
void main() {
	OPTION_REG =0b10111111;		// RB0 int active low
	// Enable SPI module:
	TRISC = 		0b11010110;		// SDO and SCK outputs, RC0 output
	RC0 = 0b1;
	SSPSTAT =	0b01000000;		// Sampling at middle, trans at high to low CLK transition
	SSPCON =		0b00100001;		// SPI enabled, master with Freq=Fosc/16
	// Interrupts:
	PIE1 = 		0b00001000;		// SPI interrupt enabled
	PIR2 =		0b00000000;		// Clear all peripherals interrupts
	INTCON =		0b11010000;		// Peripheral, RB0 interrupts enabled, flags cleared
	// GPPs
	TRISB =		0b11110001;		// RB1 output
	RB1 = 0b0;
	RB2 = 0b0;
	RB3 = 0b0;

	// Test routine
	__delay_ms(500);
	
	// SPI transaction:
	spi_buffer[0] = 0b00001010;			// Turn RFchip on
	RC0 = 0b0;									// CSN active
	spi_counter = 1;							// Data bytes
	spi_ready = 0;								// Manually clear
	SSPBUF = RF_W_REGISTER + RF_CONFIG;	// Write config register
	while (spi_ready == 0) {				// Wait
		__delay_us(1);
	}
	
	// Send test signal
	RB3 = spi_buffer[1]%2;
	RB2 = 0b1;
	__delay_ms(200);
	RB2 = 0b1;
	
	while (1) {
		// SPI transaction:
		// write payload
		spi_buffer[0] = 56;
		spi_buffer[1] = 44;
		spi_buffer[2] = 43;
		spi_buffer[3] = 42;
		RC0 = 0b0;									// CSN active
		spi_counter = 4;							// Data bytes
		spi_ready = 0;								// Manually clear
		SSPBUF = RF_W_TX_PAYLOAD;				// Write config register
		while (spi_ready == 0) {				// Wait
			__delay_us(1);
		}

		// Send test signal
		RB3 = spi_buffer[4]%2;
		RB2 = 0b1;
		__delay_ms(200);
		RB2 = 0b1;
	}

}

// Interrupt service routine
void interrupt ISR() {
	if (INTF) {			// External interrupt
		INTF = 0b0;
	}
	if (SSPIF) {		// SPI interrupt
		SSPIF = 0b0;
		
		spi_buffer[spi_counter] = SSPBUF;	// Store received value
		if (spi_counter > 0) {
			spi_counter--;
			SSPBUF = spi_buffer[spi_counter];	// Transmit next value
		}
		else {
			// Conclude transaction
			spi_ready = 1;
			RC0 = 0b1;
		}
	}
}
