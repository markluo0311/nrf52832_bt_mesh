# Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of Nordic Semiconductor ASA nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import intelhex
import struct
import sys


class DevicePageEntry:
    BL_INFO_TYPE_INVALID            = 0x00
    BL_INFO_TYPE_ECDSA_PUBLIC_KEY   = 0x01
    BL_INFO_TYPE_VERSION            = 0x02
    BL_INFO_TYPE_JOURNAL            = 0x03
    BL_INFO_TYPE_FLAGS              = 0x04

    BL_INFO_TYPE_SEGMENT_SD         = 0x10
    BL_INFO_TYPE_SEGMENT_BL         = 0x11
    BL_INFO_TYPE_SEGMENT_APP        = 0x12

    BL_INFO_TYPE_LAST               = 0x7FFF

    def __init__(self, infotype, data):
        self.infotype = infotype
        self.data = data
        pad = 4 - len(self.data) % 4
        if pad != 4:
            for p in range(pad):
                self.data += b'\xff' #struct.pack('B', [0xFF] * pad)

    def as_binary(self):
        return struct.pack('<HH', int((len(self.data) + 4) / 4), self.infotype) + self.data

def parse_hexstring(string):
    ints = [int(string[i:i+2], 16) for i in range(0, len(string), 2)]
    # python 2 and python 3 handles bytestrings differently:
    if sys.version_info < (3,0):
        return ''.join([chr(i) for i in ints])
    else:
        return bytes(ints)

class DevicePage:

    def __init__(self):
        self.entries = []
        self.declarations_file = ''


    def parse(self, declarations_file):
        entries = {}
        self.declarations_file = declarations_file
        with open(declarations_file, 'r') as f:
            for line in f:
                if line.strip().startswith('#') or not ':' in line:
                    continue
                (key, val) = tuple([part.strip() for part in line.strip().split(':')])
                try:
                    entries[key.upper()] = int(val, 0)
                except ValueError:
                    entries[key.upper()] = val


        # Construct the various entries from parts:
        key_data = None
        if 'PUBLIC_KEY' in entries:
            key_data = parse_hexstring(entries['PUBLIC_KEY'])
        elif 'VERIFICATION KEY QX' in entries and 'VERIFICATION KEY QY' in entries:
            key_data = parse_hexstring(entries['VERIFICATION KEY QX'] + entries['VERIFICATION KEY QY'])

        if key_data:
            self.entries.append(DevicePageEntry(DevicePageEntry.BL_INFO_TYPE_ECDSA_PUBLIC_KEY, key_data))


        app_segment = struct.pack('<II', entries['APP_START'], entries['APP_SIZE'])
        sd_segment  = struct.pack('<II', entries['SD_START'], entries['SD_SIZE'])
        bl_segment  = struct.pack('<II', entries['BL_START'], entries['BL_SIZE'])
        self.entries.append(DevicePageEntry(DevicePageEntry.BL_INFO_TYPE_SEGMENT_APP, app_segment))
        self.entries.append(DevicePageEntry(DevicePageEntry.BL_INFO_TYPE_SEGMENT_SD, sd_segment))
        self.entries.append(DevicePageEntry(DevicePageEntry.BL_INFO_TYPE_SEGMENT_BL, bl_segment))

        version_data = struct.pack('<HBBIHI',
                            entries['SD_VERSION'],
                            entries['BL_ID'],
                            entries['BL_VERSION'],
                            entries['COMPANY_ID'],
                            entries['APP_ID'],
                            entries['APP_VERSION'])
        self.entries.append(DevicePageEntry(DevicePageEntry.BL_INFO_TYPE_VERSION, version_data))

        self.entries.append(DevicePageEntry(DevicePageEntry.BL_INFO_TYPE_FLAGS, struct.pack('I', 0xFFFFFFFF)))


    def generate_hex(self, start_addr = 0x3fc00, hexfile = None):
        if hexfile == None:
            hexfile = self.declarations_file + '.hex'
        binary_string = struct.pack('<BBBB', 4, 1, 8, 8) # info page metadata
        for entry in self.entries:
            binary_string += entry.as_binary()
        binary_string += struct.pack('<HH', 0xFFFF, DevicePageEntry.BL_INFO_TYPE_LAST)

        # python 2 and python 3 handles bytestrings differently:
        if sys.version_info < (3,0):
            binary_string = [ord(x) for x in binary_string]

        hexoutput = intelhex.IntelHex()
        hexoutput.frombytes(binary_string, start_addr)
        hexoutput.tofile(hexfile, 'hex')

if __name__ == '__main__':
    if len(sys.argv) != 2 or (len(sys.argv) == 3 and not '--nrf52' in sys.argv):
        print('Usage: python device_page.py <info file> [--nrf52]')
        exit(1)
    devpage = DevicePage()
    devpage.parse(sys.argv[1])
    if '--nrf52' in sys.argv:
        devpage.generate_hex(start_addr = 0x7f000)
    else:
        devpage.generate_hex(start_addr = 0x3fc00)


