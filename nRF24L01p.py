#!/usr/bin/python
# 
# Library for ISP communication with nRF24L01+ for raspberrypi.
# Uses spidev: https://github.com/doceme/py-spidev
# For a higher level library, see https://github.com/jpbarraca/pynrf24/blob/master/nrf24.py

import time
import spidev
import RPi.GPIO as gpio

class nRF24L01p:

    # Instruction Mnemonics
    R_REGISTER = 0x00
    W_REGISTER = 0x20
    R_RX_PAYLOAD = 0x61
    W_TX_PAYLOAD = 0xA0
    FLUSH_TX = 0xE1
    FLUSH_RX = 0xE2
    REUSE_TX_PL = 0xE3
    R_RX_PL_WID = 0x60
    W_ACK_PAYLOAD = 0xA8
    W_TX_PAYLOAD_NOACK = 0xB0
    NOP = 0xFF

    # Register addresses
    CONFIG = 0x00
    EN_AA = 0x01
    EN_RXADDR = 0x02
    SETUP_AW = 0x03
    SETUP_RETR = 0x04
    RF_CH = 0x05
    RF_SETUP = 0x06
    STATUS = 0x07
    OBSERVE_TX = 0x08
    RPD = 0x09
    RX_ADDR_P0 = 0x0A
    RX_ADDR_P1 = 0x0B
    RX_ADDR_P2 = 0x0C
    RX_ADDR_P3 = 0x0D
    RX_ADDR_P4 = 0x0E
    RX_ADDR_P5 = 0x0F
    TX_ADDR = 0x10
    RX_PW_P0 = 0x11
    RX_PW_P1 = 0x12
    RX_PW_P2 = 0x13
    RX_PW_P3 = 0x14
    RX_PW_P4 = 0x15
    RX_PW_P5 = 0x16
    FIFO_STATUS = 0x17
    DYNPD = 0x1C
    FEATURE = 0x1D

    def __init__(self, spi_device=0, CE_port=24, IRQ_port=25):
        """Initialize spi communication with device. IRQ and CE pins are set in BCM mode.
            Note: CE0 and CE1 are selected via spi_device, and connect to CSN."""
        self.spi = spidev.SpiDev()
        self.spi.open( 0, 0)
        self.spi.bits_per_word = 8
        self.spi.max_speed_hz = 10000000
        self.CE = CE_port
        self.IRQ = IRQ_port
        gpio.setmode(gpio.BCM)
        gpio.setup( self.CE, gpio.OUT, initial=gpio.LOW)
        gpio.setup( self.IRQ, gpio.IN)

    def __del__(self):
        self.spi.close()
        gpio.cleanup()

    def read_register(self, address):
        """Read command and status registers."""
        data = [self.R_REGISTER + address, self.NOP]
        return self.spi.xfer2(data)

    def read_register_multibyte(self, address, length):
        """Read registers RX_ADDR_P0, RX_ADDR_P1 and TX_ADDR."""
        data = [self.R_REGISTER + address] + ([self.NOP] * (length))
        return self.spi.xfer2(data)

    def write_register(self, address, value):
        """Write command and status registers."""
        data = [self.W_REGISTER + address, value]
        return self.spi.xfer2(data)[0]

    def write_register_multibyte(self, address, values):
        """Write registers RX_ADDR_P0, RX_ADDR_P1 and TX_ADDR."""
        data = [self.W_REGISTER + address] + values
        return self.spi.xfer2(data)[0]

    def read_payload(self, length):
        """Read RX-payload: 1 - 32 bytes. A read operation
            always starts at byte 0. Payload is deleted from
            FIFO after it is read. Used in RX mode."""
        data = [self.R_RX_PAYLOAD] + ([self.NOP] * (length))
        return self.spi.xfer2(data)[0]

    def write_payload(self, values):
        """Write TX-payload: 1 - 32 bytes. A write operation
            always starts at byte 0 used in TX payload."""
        data = [self.W_TX_PAYLOAD] + values
        return self.spi.xfer2(data)[0]

    def flush_tx(self):
        """Flush TX FIFO, used in TX mode."""
        return self.spi.xfer2( [self.FLUSH_TX])[0]

    def flush_rx(self):
        """Flush RX FIFO, used in RX mode
            Should not be executed during transmission of
            acknowledge, that is, acknowledge package will
            not be completed."""
        return self.spi.xfer2( [self.FLUSH_RX])[0]

    def reuse_payload(self):
        """Used for a PTX device
            Reuse last transmitted payload.
            TX payload reuse is active until
            W_TX_PAYLOAD or FLUSH TX is executed. TX
            payload reuse must not be activated or deacti-
            vated during package transmission."""
        return self.spi.xfer2( [self.REUSE_TX_PL])[0]

    def read_payload_length(self):
        """Read RX payload width for the top
            R_RX_PAYLOAD in the RX FIFO.
            Note: Flush RX FIFO if the read value is larger
            than 32 bytes"""
        return self.spi.xfer2( [self.R_RX_PL_WID])

    def write_payload_pipe(self, pipe, values):
        """Used in RX mode.
            Write Payload to be transmitted together with
            ACK packet on PIPE pipe. (pipe valid in the
            range from 0 to 5). Maximum three ACK
            packet payloads can be pending. Payloads with
            same pipe are handled using first in - first out
            principle. Write payload: 1- 32 bytes. A write
            operation always starts at byte 0."""
        data = [self.W_ACK_PAYLOAD + pipe] + values
        return self.spi.xfer2(data)[0]

    def write_payload_noack(self, values):
        """Used in TX mode. Disables AUTOACK on this
            specific packet."""
        data = [self.W_TX_PAYLOAD_NOACK] + values
        return self.spi.xfer2(data)[0]

    def read_status(self):
        """Read the STATUS register without performing
            any operation."""
        return self.spi.xfer2( [self.NOP])[0]

    def enable(self):
        """Enable device, send or receive packets depending on PRIM_RX."""
        gpio.output( self.CE, gpio.HIGH)

    def disable(self):
        """Disable device, will enter Standby-I mode."""
        gpio.output( self.CE, gpio.LOW)

    def test(self):
        print("Turn on:")
        print(bin( self.write_register(self.CONFIG, 0b00001010) ))
        print("Write payload")
        print(bin( self.write_payload( [42, 43, 44, 56] ) ))
        print("Enable device")
        self.enable()
        time.sleep(0.0001)
        self.disable()
        print("Status after disabling:")
        print(bin( self.read_status() ))

