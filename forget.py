#!/usr/bin/env python3
# forget: download bitstream to Renesas ForgeFPGA SLG47910 RAM using
#         Excamera SPIDriver USB-to-SPI interface
# Copyright 2022 Eric Smith <spacewar@gmail.com>
# SPDX-License-Identifier: GPL-3.0-only

import argparse
import struct
import time

import spidriver


def bitreverse_byte(b):
    b = ((b & 0x55) << 1) | ((b & 0xaa) >> 1)
    b = ((b & 0x33) << 2) | ((b & 0xcc) >> 2)
    b = ((b & 0x0f) << 4) | ((b & 0xf0) >> 4)
    return b

bitreverse_byte_table = [bitreverse_byte(b) for b in range(256)]

def bitreverse_bytes(d):
    return bytes([bitreverse_byte(b) for b in d])


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
        self._spi.write(bitreverse_bytes(data))



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
