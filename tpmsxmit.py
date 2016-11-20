#!/usr/bin/env python2
# -*- coding: utf-8 -*-

#
# WJ Pacific TPMS encoder
#
# Copyright (C) 2016 Werner Johansson, wj@xnk.nu
# Author: Werner Johansson <wj@xnk.nu>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import osmosdr
import time
import crcmod

# From the TPSM code
def split_string_bytes(data, start_offset):
        for n in range(start_offset, len(data), 8):
                yield data[n:n+8]

def differential_manchester_encode(s):
        last_bit = 0
        state = 0
        result = []
        for sbit in s:
                bit = 0
                if sbit == '1': bit = 1
                if bit == last_bit:
                        result.append('10')
                        state=0
                else:
                        result.append('01')
                        state=1
                last_bit = state
        return ''.join(result)

class toyota_tpms_tx(gr.top_block):
    def __init__(self):
        gr.top_block.__init__(self)
        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 10000000
        self.bitrate = bitrate = 9910.0

        ##################################################
        # Blocks
        ##################################################
        self.osmosdr_sink_0 = osmosdr.sink( args="numchan=" + str(1) + " " + '' )
        self.osmosdr_sink_0.set_sample_rate(samp_rate)
        self.osmosdr_sink_0.set_center_freq(314.98e6, 0)
        self.osmosdr_sink_0.set_freq_corr(4, 0)
        self.osmosdr_sink_0.set_gain(14, 0)
        self.osmosdr_sink_0.set_if_gain(20, 0)
        self.osmosdr_sink_0.set_bb_gain(20, 0)
        self.osmosdr_sink_0.set_antenna('', 0)
        self.osmosdr_sink_0.set_bandwidth(0, 0)
          
#        self.freq_xlating_fir_filter_xxx_0 = filter.freq_xlating_fir_filter_ccc(10, (firdes.low_pass(1, samp_rate, 180000, 20000, firdes.WIN_HAMMING, 6.76)), 0, samp_rate)
        self.digital_gfsk_mod_0 = digital.gfsk_mod(
        	samples_per_symbol=samp_rate/bitrate/8,
        	sensitivity=0.050,
        	bt=0.9,
        	verbose=False,
        	log=False,
        )
        self.blocks_vector_source_x_0 = blocks.vector_source_b([0], False, 1, [])
#        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_gr_complex*1, 'testgen_314.97m_0.400m_20161115_133333z_hackrf.cfile', False)
#        self.blocks_file_sink_0.set_unbuffered(True)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_vector_source_x_0, 0), (self.digital_gfsk_mod_0, 0))    
#        self.connect((self.digital_gfsk_mod_0, 0), (self.freq_xlating_fir_filter_xxx_0, 0))    
        self.connect((self.digital_gfsk_mod_0, 0), (self.osmosdr_sink_0, 0))    
#        self.connect((self.freq_xlating_fir_filter_xxx_0, 0), (self.blocks_file_sink_0, 0))    

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.osmosdr_sink_0.set_sample_rate(self.samp_rate)
        self.freq_xlating_fir_filter_xxx_0.set_taps((firdes.low_pass(1, self.samp_rate, 180000, 20000, firdes.WIN_HAMMING, 6.76)))

    def get_bitrate(self):
        return self.bitrate

    def set_bitrate(self, bitrate):
        self.bitrate = bitrate


def main(top_block_cls=toyota_tpms_tx, options=None):

    crc8 = crcmod.mkCrcFun(0x113, rev=False, initCrc=0, xorOut=0)

    # TPMS sensor 7 hex digit ID as printed on the sensor
    device = format(0xabcdef7, '028b')

    # Battery low if set
    battlow = '0'

    # Counter (two bits counts 1,2,3)
    ctr = format(0x3, '02b')

    # Must be zero for packet to be recognized
    mustbezero = '0'
    # Ignored bit, sensor sets this to 0
    ignored = '0'
    # Fault indicator of some sort (if 1 TPMS light starts flashing and values are ignored)
    fault = '0'

    # Tire pressure in PSI/0.363 + 40 constant added or
    # kPa/2.48 + 40 constant added
    pressure = 42/0.363 + 40
    pressureint = int(pressure + 0.5)
    pressurebits = format(pressureint, '08b')
    pressureinvbits = format(pressureint^0xff, '08b')

    # Temperature in Celsius with a constant 40 added (resulting range -40 to 215C)
    temperature = format(42 + 40, '08b')
    #temperature = '00110001'

    # Assemble the complete payload to calculate CRC
    payload = device + battlow + ctr + mustbezero + ignored + fault + pressurebits + pressureinvbits + temperature

    # Pad payload to make it even 8 bits (won't affect CRC8 as long as it's done in front)
    padpayload = '000000'+payload
    crc_bytes_str = tuple(split_string_bytes(padpayload, 0))
    crc_bytes = map(lambda v: int(v, 2), crc_bytes_str)
    crc_str = ''.join(map(chr, crc_bytes))
    calculated_crc = crc8(crc_str[0:8])
    crc_bits = format(calculated_crc, '08b')

    bitstream = '00000000000000001111110' + differential_manchester_encode('1'+payload+crc_bits+'1') + '000000'
    print('bitstream = %s' % (bitstream))

    #bytevect = []
    # In case transmitter doesn't start up immediately
    bytevect = [0] * 4000
    for x in range(0,15):
        for iter in bitstream:
            bytevect.append(int(iter)*255)

        bytevect += [0] * 1000

    tb = top_block_cls()
    tb.blocks_vector_source_x_0.set_data(bytevect)
    tb.run()

if __name__ == '__main__':
    main()
