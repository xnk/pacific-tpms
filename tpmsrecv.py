#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# WJ Pacific TPMS decoder
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

if __name__ == '__main__':
    import ctypes
    import sys

from gnuradio import analog
from gnuradio import filter
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import math
import numpy
import osmosdr
import crcmod

usefile=0

def decode_pacific(payload):
        crc8 = crcmod.mkCrcFun(0x113, rev=False, initCrc=0, xorOut=0)
        decoded_data = []
        crcfails = 0

        # Pack bits into bytes in two ways, aligned to the end and beginning of message
        padpayload = '000000'+payload
        crc_bytes = int(padpayload, 2).to_bytes(len(padpayload) // 8, byteorder='big')

        payload_bytes = int(payload + '000000', 2).to_bytes(len(padpayload) // 8, byteorder='big')

        packet_crc = crc_bytes[8]
        calculated_crc = crc8(crc_bytes[0:8])
        crc_ok = (calculated_crc == packet_crc) and (0xff == (crc_bytes[5] ^ crc_bytes[6]))

        pressure = 2.48 * (crc_bytes[5] - 40)
        temperature = crc_bytes[7] - 40

        if crc_ok:
                print('ID %02x%02x%02x%01x BatLo%s Ctr%d Unk%s Fail%s %5.1fkPa %3dC' % (
                        payload_bytes[0], payload_bytes[1], payload_bytes[2], payload_bytes[3]>>4,
                        payload[28:29], (payload_bytes[3]&0x6)>>1, payload[31:33], payload[33:34], pressure, temperature
                ))
        else:
                print('CRC error, not decoding (%s) (%s)', payload, payload_bytes.hex())


class tpmsrx(gr.sync_block):
    
    def __init__(self):
        gr.sync_block.__init__(self,
            name="tpmsrx",
            in_sig=[numpy.int8],
            out_sig=[])
        self.state=-1
        self.pendingbit=0
        self.lastdiffbit=1
        self.pktbuild=[]

    def work(self, input_items, output_items):
        for bit in numpy.nditer(input_items[0], order='K'):
            if bit&2:
#                print 'pktbegin'
                self.pktbuild=[]
                self.state=0
                self.lastdiffbit=1
                bit = bit & 1
            if self.state>=0:
                if self.state&1:
                    # Differential manchester decoding
                    if bit != self.pendingbit:
                        if self.lastdiffbit != self.pendingbit:
                            self.pktbuild.append('0')
                        else:
                            self.pktbuild.append('1')
                        self.lastdiffbit = bit
                        self.state += 1
                        if self.state>=(66*2):
                            #print('[%s]' % (''.join(self.pktbuild)))
                            decode_pacific(''.join(self.pktbuild))
                            self.state=-1
                    else:
#                        print('Manchester violation after %d bits' % (self.state))
                        self.state=-1
                else:
                    self.pendingbit = bit
                    self.state += 1
        return len(input_items[0])

class top_block(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self)

        ##################################################
        # Variables
        ##################################################
        self.throttle_rate = throttle_rate = 4000000
        self.samp_rate = samp_rate = 400000
        self.samp_per_sym = samp_per_sym = samp_rate/9910.0
        self.fsk_deviation_hz = fsk_deviation_hz = 41000

        ##################################################
        # Blocks
        ##################################################
        if usefile:
            self.blocks_file_source_0 = blocks.file_source(gr.sizeof_gr_complex*1, 'driverecording400kHz.cfile', False)
            # Adjust offset depending on where carrier is
            self.freqadj=10000
            self.decim=1
            throttle_rate=400000
        else:
            self.osmosdr_source_0 = osmosdr.source( args="numchan=" + str(1) + " " + '' )
            self.osmosdr_source_0.set_sample_rate(throttle_rate)
            self.osmosdr_source_0.set_center_freq(316e6, 0)
            self.osmosdr_source_0.set_freq_corr(4, 0)
            self.osmosdr_source_0.set_dc_offset_mode(0, 0)
            self.osmosdr_source_0.set_iq_balance_mode(0, 0)
            self.osmosdr_source_0.set_gain_mode(False, 0)
            self.osmosdr_source_0.set_gain(14, 0)
            self.osmosdr_source_0.set_if_gain(30, 0)
            self.osmosdr_source_0.set_bb_gain(20, 0)
            self.osmosdr_source_0.set_antenna('', 0)
            self.osmosdr_source_0.set_bandwidth(0, 0)
            self.freqadj=-1.015e6
            self.decim=10

        self.freq_xlating_fir_filter_xxx_0 = filter.freq_xlating_fir_filter_ccc(self.decim, (firdes.low_pass(1, throttle_rate, 180000, 20000, firdes.WIN_HAMMING, 6.76)), self.freqadj, throttle_rate)

        self.low_pass_filter_0 = filter.fir_filter_fff(1, firdes.low_pass(1, samp_rate, 8000, 5000, firdes.WIN_HAMMING, 6.76))
        self.digital_correlate_access_code_bb_0 = digital.correlate_access_code_bb('000111111001', 0)
        self.digital_clock_recovery_mm_xx_0 = digital.clock_recovery_mm_ff(samp_per_sym*(1+0.0), 0.25*0.175*0.175, 0.5, 0.175, 0.005)
        self.digital_binary_slicer_fb_0 = digital.binary_slicer_fb()
        self.blocks_tpmsrx_sink_0 = tpmsrx()
        self.analog_quadrature_demod_cf_0 = analog.quadrature_demod_cf(samp_rate/(2*math.pi*fsk_deviation_hz/8.0))

        ##################################################
        # Connections
        ##################################################
        self.connect((self.freq_xlating_fir_filter_xxx_0, 0), (self.analog_quadrature_demod_cf_0, 0))
        if usefile:
                self.connect((self.blocks_file_source_0, 0), (self.freq_xlating_fir_filter_xxx_0, 0))    
        else:
                self.connect((self.osmosdr_source_0, 0), (self.freq_xlating_fir_filter_xxx_0, 0))
        self.connect((self.analog_quadrature_demod_cf_0, 0), (self.low_pass_filter_0, 0))    
        self.connect((self.low_pass_filter_0, 0), (self.digital_clock_recovery_mm_xx_0, 0))    
        self.connect((self.digital_binary_slicer_fb_0, 0), (self.digital_correlate_access_code_bb_0, 0))    
        self.connect((self.digital_clock_recovery_mm_xx_0, 0), (self.digital_binary_slicer_fb_0, 0))    
        self.connect((self.digital_correlate_access_code_bb_0, 0), (self.blocks_tpmsrx_sink_0, 0))    

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_samp_per_sym(self.samp_rate/9910)
        self.analog_quadrature_demod_cf_0.set_gain(self.samp_rate/(2*math.pi*self.fsk_deviation_hz/8.0))

    def get_samp_per_sym(self):
        return self.samp_per_sym

    def set_samp_per_sym(self, samp_per_sym):
        self.samp_per_sym = samp_per_sym
        self.digital_clock_recovery_mm_xx_0.set_omega(self.samp_per_sym*(1+0.0))

    def get_fsk_deviation_hz(self):
        return self.fsk_deviation_hz

    def set_fsk_deviation_hz(self, fsk_deviation_hz):
        self.fsk_deviation_hz = fsk_deviation_hz
        self.analog_quadrature_demod_cf_0.set_gain(self.samp_rate/(2*math.pi*self.fsk_deviation_hz/8.0))


def main(top_block_cls=top_block, options=None):

    tb = top_block_cls()
    tb.run()


if __name__ == '__main__':
    main()
