#!/usr/bin/env python3
# forget: download bitstream to Renesas ForgeFPGA SLG47910 RAM using
#         Excamera SPIDriver USB-to-SPI interface
# Copyright 2022 Eric Smith <spacewar@gmail.com>
# SPDX-License-Identifier: GPL-3.0-only

import argparse
import struct
import time

import spidriver


# The Renesas ForgeFPGA Configuration Guide Rev 1.0 dated May 31, 2022 has
# a number of ambiguities.

# In section 4, "General SPI Interface", they talk about the four
# possible SPI modes, and give an example of mode 0 (CPHA = 0, CPOL =
# 0), but they don't anywhere actually state which mode is required by
# the ForgeFPGA chip. The mode used by the Renesas ForgeFPGA
# Evaluation Board is CPHA = 1, CPOL = 1. The Configuration Guide
# Table 2 says that is mode 2, but other sources of SPI mode
# documentation refer to that as mode 3.

# They don't explicity define what bit ordering the ForgeFPGA uses.
# Figure 3 an example SPI byte transfer with most signficant bit first.

# In section 8, "MCU Programming (Slave Mode)", in the list of programming
# steps, it is said of the sync word 0x11ff22aa "Transmission starts from
# the LSB. They mean tha the bytes are ordered starting with the least
# significant byte (0xaa), but the bit order is still most signficant bit
# (of a byte) first.

# When the Renesas develo0pment software configures the FPGA on the
# evaluation board, it first performs a single-word SPI transaction
#   * assert SPI slave select
#   * send one 32-bit word 0x55aa55aa, least significant byte first
#   * deassert SPI slave select
# This is not described in section 8. The purpose is unknown. Whether it
# is required is unknown.


class ForgeFPGA:
    def __init__(self, bitstream):
        self._bitstream_length_bytes = 11264 * 4
        if len(bitstream) != self._bitstream_length_bytes:
            raise Exception(f'bitstream length {len(bitstream)} bytes, should be {self._bitstream_length_bytes} bytes')
        self._bitstream = bitstream

        self._preamble_byte_count = 321 * 4
        self._sync_bytes = struct.pack('<L', 0x11ff22aa)
        self._regs_byte_count = 9 * 4  # "other SOC registers" "Default all zeros"
        self._postamble_byte_count = 6 * 4

    def download_to_ram(self, spi: spidriver.SPIDriver):
        self._spi = spi

        self._spi.setmode(3)

	# FPGA SPI /SS needs to be low at power-up to enable
        # SPI slave mode configuration
        self._set_fpga_power_and_enable(False)
        self._spi_select(True)
        time.sleep(0.05)  # give thee FPGA some time to wake up
        self._set_fpga_power_and_enable(True)

        self._send_zero_bytes(self._preamble_byte_count)
        self._send_bytes(self._sync_bytes)
        self._send_zero_bytes(self._regs_byte_count)
        self._send_bytes(self._bitstream)
        self._send_zero_bytes(self._postamble_byte_count)
        
        self._spi_select(False)
        self._spi = None

    def _set_fpga_power_and_enable(self, state: bool):
        self._spi.seta(int(state))
        self._spi.setb(int(state))

    def _spi_select(self, state: bool):
        if state:
            self._spi.sel()
        else:
            self._spi.unsel()

    def _send_zero_bytes(self, byte_count):
        zeros = bytes(byte_count)
        self._spi.write(zeros)

    def _send_bytes(self, data: bytes):
        self._spi.write(data)



def main():
    parser = argparse.ArgumentParser(description = 'Download ForgeFPGA configuration to RAM.')

    parser.add_argument('bitstream',
                        type = argparse.FileType('rb'),
                        help = 'bistream file')

    # It would be nice to automatically detect the SPIDriver device file,
    # /dev/ttyUSB<n> on Linux, but it just uses an FTDI chip with no special
    # programming (USB VID 0403, PID 6015), so it's not easily distinguishable
    # from many other devices using USB FTDI chips.
    parser.add_argument('--spidriver', '-s',
                        type = str,
                        default = '/dev/ttyUSB0',
                        help = 'SPIDriver USB device file')

    args = parser.parse_args()

    bitstream = args.bitstream.read()

    spi = spidriver.SPIDriver(args.spidriver)
    if False:
        print(f'{spi.product=}')
        print(f'{spi.serial=}')
        print(f'{spi.uptime=}')
        print(f'{spi.voltage=}')
        print(f'{spi.current=}')
        print(f'{spi.temp=}')

    forgefpga = ForgeFPGA(bitstream)
    forgefpga.download_to_ram(spi)


if __name__ == '__main__':
    main()
