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


import json
import glob

HEADER_START="""/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PACKET_MESH_H__
#define PACKET_MESH_H__

/**
 * Generic mesh/advertising packet.
 */
typedef struct __attribute((packed))
{
    uint8_t length;  /**< The length of the ad_type + pdu.*/
    uint8_t ad_type; /**< Advertising type. */
    uint8_t pdu[];   /**< Mesh PDU. */
} packet_mesh_t;
"""
HEADER_END="#endif"

class Field(object):
    """Packet field object."""
    def __init__(self, name, module, width, doc, visible, bit_offset=0):
        """Initializes a field for a packet.

        name    -- the (short) name of the field
        module  -- the (short) module that owns the field
        width   -- the width (in bits) of the field
        doc     -- a docstring that makes sense in the following sentence: 'returns the #doc#'
        visible -- True if this field should generate getter/setter functions.
        """
        self.name    = name
        self.module  = module
        self.width   = width
        self.doc     = doc
        self.visible = visible

        self._bit_offset = bit_offset
        self._masks      = []
        self._masks_inv  = []
        self._offsets    = []
        self.make_masks()
        self.make_offsets()

    def __str__(self):
        return "{:10}: {:>2}  -- {}".format(self.name,
                                            str(self.width),
                                            self.doc[0].upper() + self.doc[1:])

    def __eq__(self, other):
        return self.name == other.name

    def make_masks(self):
        """Makes the masks to access the bitfield variable."""
        if not self.is_crossing_byte_boundary():
            self._masks = [self.get_mask(self._bit_offset, self.width)]
        else:
            bit_offset = self._bit_offset
            width = 8 - bit_offset % 8
            self._masks.append(self.get_mask(bit_offset, width))

            bit_offset += width
            width = self.width - width

            # If the bitfield spans across multiple bytes:
            while width > 8:
                self._masks.append(self.get_mask(bit_offset, 8))
                bit_offset += 8
                width      -= 8

            # Last part
            self._masks.append(self.get_mask(bit_offset, width))

        # Make the inverse masks
        for mask in self._masks:
            self._masks_inv.append((~mask) & 0xFF)

    def make_offsets(self):
        """Makes the byte index offsets to find the field in the pdu[] array."""
        if not self.is_crossing_byte_boundary():
            self._offsets = [self._bit_offset // 8]
        elif self.width <= 8:
            self._offsets = list(range(self._bit_offset // 8,
                                       self._bit_offset // 8 + 2))
        else:
            self._offsets = list(range(self._bit_offset // 8,
                                       self._bit_offset // 8
                                       + (self.width - 1) // 8
                                       + 1))

    def get_bitmask_defines(self):
        """Returns the bitmask defines as a string."""
        defs = ""
        if len(self._masks) == 1 and self._masks[0] != 0xFF:
            defs  = "#define PACKET_MESH_%s_MASK     (0x%02X)" % \
                    (self.name.upper(), self._masks[0])

            defs += "    /**< Mask for %s field. */\n" % (self.name.lower())
            defs += "#define PACKET_MESH_%s_MASK_INV (0x%02X)" % \
                    (self.name.upper(), self._masks_inv[0])
            defs += "    /**< Inverse mask for %s field. */\n" % (self.name.lower())
        else:
            defs  = ["#define PACKET_MESH_%s%u_MASK     (0x%02X)" %   \
                     (self.name.upper(), i, self._masks[i])         + \
                     "    /**< Mask for %s field (%u). */" % (self.name.lower(), i) \
                     for i in range(len(self._masks)) if self._masks[i] != 0xFF]
            defs += ["#define PACKET_MESH_%s%u_MASK_INV (0x%02X)" % \
                     (self.name.upper(), i, self._masks_inv[i])   + \
                     "    /**< Inverse mask for %s field (%u). */" % (self.name.lower(), i) \
                     for i in range(len(self._masks)) if self._masks[i] != 0xFF]
            defs = "\n".join(defs) + "\n"
        if defs.strip() == "":
            return ""
        else:
            return defs

    def get_byte_offset_defines(self):
        """Returns the byte offset defines corresponding to the bitmasks as a string."""
        defs = ""
        if len(self._offsets) == 1:
            ws = (8 - len(str(self._offsets[0]))) * " "
            defs = "#define PACKET_MESH_%s_OFFSET   (%u)" % \
                   (self.name.upper(), self._offsets[0])
            defs  += ws + "/**< Offset to the %s field.*/\n" % (self.name.lower())
        else:
            defs = []
            for i in range(len(self._offsets)):
                ws = (8 - len(str(self._offsets[i]))) * " "
                defs += ["#define PACKET_MESH_%s%u_OFFSET   (%u)"   % (self.name.upper(), i, self._offsets[i]) + \
                         ws + "/**< Offset to the %s field (%u).*/" % (self.name.lower(), i)]
            defs  = "\n".join(defs) + "\n"
        return defs

    def get_getter_function(self):
        """Gets the getter function for this field."""
        NUM_SPACES = 4
        ws = " " * NUM_SPACES
        ctype = self.get_ctype((self.width - 1) // 8)

        fun = ["""/**
 * Gets the %s.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the %s.
 */""" % (self.doc, self.doc)]

        fun += ["static inline %s packet_mesh_%s_%s_get(const packet_mesh_t * p_pkt)" \
                % (ctype, self.module.lower(), self.name.lower()),
                "{"]

        if self.width == 1:
            fun.append(ws + "return (" + self.get_str(0) + " > 0);")
        elif len(self._masks) == 1:
            # lsb_offset is the offset from bit 0 to the start of the field.
            # E.g., the field could be spanning from bit 2->4 and needs to be shifted down.
            lsb_offset = (8 - (self._bit_offset + self.width) % 8)
            if lsb_offset % 8 > 0:
                fun.append(ws + "return (" + self.get_str(0) + ">> %u );" % (lsb_offset))
            else:
                fun.append(ws + "return " + self.get_str(0) + ";")
        elif self.is_crossing_byte_boundary() and self.width <= 8:
            # I.e.: +------+------+
            #       |  <by | te>  |
            #       +------+------+
            bitshift = (self.width + self._bit_offset) % 8
            fun.append(ws + "return ((" + self.get_str(0) + " << %u) |" % (bitshift))
            ws += len("return (") * " "
            fun.append(ws + " " + self.get_str(1) + " >> %u);" % (8 - bitshift))
        else:
            bitshift = ((self.width - 1) // 8) * 8
            lsb_offset = (8 - (self._bit_offset + self.width) % 8)
            if lsb_offset % 8 > 0:
                bitshift -= lsb_offset

            fun.append(ws + "return ((" + self.get_str(0) + " << %u) | " % (bitshift))
            ws += len("return (") * " "
            for i in range(1, len(self._masks) - 1):
                bitshift = ((self.width // 8) - i - 1) * 8
                fun.append(ws + "(" + self.get_str(i) + " << %u) |" % (bitshift))

            final_offset = (self._bit_offset + self.width) % 8
            if final_offset > 0:
                lsb_offset = 8 - final_offset
                fun.append(ws + "(" + self.get_str(-1) + " >> %u));" % (lsb_offset))
            else:
                fun.append(ws + self.get_str(-1) + ");")
        fun.append("}")
        return "\n".join(fun) + "\n"

    def get_setter_function(self):
        """Gets the setter function for this field."""
        NUM_SPACES = 4
        ws = " " * NUM_SPACES
        ctype = self.get_ctype((self.width - 1) // 8)
        if len(self._masks) > 1:
            names = [self.name.upper() + str(i) for i in range(len(self._masks))]
        else:
            names = [self.name.upper()]


        fun = ["""/**
 * Sets the %s.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the %s.
 */""" % (self.doc, self.doc)]

        fun += ["static inline void packet_mesh_%s_%s_set(packet_mesh_t * p_pkt, %s val)" \
                % (self.module.lower(), self.name.lower(), ctype),
                "{"]

        for i in range(len(self._masks)-1):

            if self.is_crossing_byte_boundary() and self.width <= 8:
                bitshift = (self.width + self._bit_offset) % 8
            else:
                bitshift = ((self.width - 1) // 8 - i)* 8
                lsb_offset = (8 - (self._bit_offset + self.width) % 8)
                if lsb_offset % 8 > 0:
                    bitshift -= lsb_offset

            if self._masks[i] == 0xFF:
                fun.append(ws
                           + "p_pkt->pdu[PACKET_MESH_%s_OFFSET] = (val >> %u) & 0xFF;" \
                           % (names[i], bitshift))
            else:
                fun.append(ws
                           + "p_pkt->pdu[PACKET_MESH_%s_OFFSET] &= PACKET_MESH_%s_MASK_INV;" \
                           % (names[i], names[i]))
                fun.append(ws
                           + "p_pkt->pdu[PACKET_MESH_%s_OFFSET] |= (val >> %u) & PACKET_MESH_%s_MASK;" \
                           % (names[i], bitshift, names[i]))

        if self._masks[-1] == 0xFF:
            fun.append(ws + "p_pkt->pdu[PACKET_MESH_%s_OFFSET] = val & 0xFF;" % (names[-1]))
        else:
            fun.append(ws
                       + "p_pkt->pdu[PACKET_MESH_%s_OFFSET] &= PACKET_MESH_%s_MASK_INV;" \
                       % (names[-1], names[-1]))

            lsb_offset = 8 - (self._bit_offset + self.width) % 8
            if lsb_offset % 8 > 0:
                fun.append(ws
                           + "p_pkt->pdu[PACKET_MESH_%s_OFFSET] |= (val << %u) & PACKET_MESH_%s_MASK;" \
                           % (names[-1], lsb_offset, names[-1]))
            else:
                fun.append(ws
                           + "p_pkt->pdu[PACKET_MESH_%s_OFFSET] |= (val & PACKET_MESH_%s_MASK);" \
                           % (names[-1], names[-1]))

        fun.append("}")
        return "\n".join(fun) + "\n"


    def get_definitions(self):
        """Joins the bitmask and offset defines and returns them as a string."""
        if not self.visible:
            return ""
        return (self.get_byte_offset_defines() + self.get_bitmask_defines()).strip() + "\n"

    def get_functions(self):
        """Joins the setter and getter functions and returns them as a string."""
        if not self.visible:
            return ""
        return (self.get_getter_function() + "\n" + self.get_setter_function()).strip() + "\n"

    def get_str(self, i):
        """Gets getter-mask given mask index."""
        if i == -1:
            i = len(self._masks)-1
        if len(self._masks) > 1:
            name = self.name.upper() + str(i)
        else:
            name = self.name.upper()
        if self._masks[i] == 0xFF:
            return "p_pkt->pdu[PACKET_MESH_%s_OFFSET]" % (name)
        else:
            return "(p_pkt->pdu[PACKET_MESH_%s_OFFSET] & PACKET_MESH_%s_MASK)" % (name, name)

    @staticmethod
    def get_ctype(byte_width):
        CTypes = {0: "uint8_t", 1: "uint16_t", 2: "uint32_t", 3: "uint32_t"}
        return CTypes[byte_width]

    @staticmethod
    def get_mask(bit_offset, width):
        """Gets the mask for accessing a field at 'bit_offset' with width 'width'."""
        return (0xff << (8 - (bit_offset % 8 + width)) &
                0xff >> bit_offset % 8)

    def is_crossing_byte_boundary(self):
        """Returns True if this field spans across one (or more) byte boundaries."""
        # Example:
        # 8        0|8        0
        # +---------+---------+
        # | f1 |< field >| f2 |
        # +---------+---------+
        return (8 - (self._bit_offset % 8)) < self.width


class PacketFMT(object):
    def __init__(self, name, type_, fields):
        self._name = name
        self._type = type_
        self._total_offset = 0
        self._fields = []
        if type(fields) is list:
            for elem in fields:
                if type(elem) is dict:
                    f = Field(**elem, bit_offset=self._total_offset)
                    if f not in self._fields:
                        self._fields += [f]
                    self._total_offset += elem["width"]
                else:
                    print(elem)
                    raise TypeError("Fields must be Field object or dict.")

    def __iadd__(self, other):
        for f in other._fields:
            if f not in self._fields:
                self._fields += [f]
        return self

    def __contains__(self, other):
        return (other in self._fields)

    def __str__(self):
        s = self._name + ":\n"
        s += "\t+" + "-" * 20 + "\n"
        for f in self._fields:
            s += "\t| " + str(f) + "\n"
        if self._type == "absolute":
            s += "\t+" + "-" * 20
        else:
            s += "\t| " + "[...]" + "\n"
            s += "\t| " + "MIC" + "\n"
            s += "\t+" + "-" * 20
        return s

    def get_fields(self):
        return list(self._fields)

def as_packet_fmt(dct):
    if "__packet__" in dct:
        return PacketFMT(dct["name"], dct["type"], dct["fields"])
    else:
        return dct

def json_reads():
    jdata = []
    for path in glob.glob("packet_fmt/packets/*.json"):
        with open(path, "r") as f:
            jdata += [json.load(f, object_hook=as_packet_fmt)]

    return jdata

if __name__ == "__main__":
    fields = json_reads()
    pdu_defines = ""
    pdu_funcs = ""
    for f in fields:
        if f._type == "absolute":
            pdu_defines += "#define PACKET_MESH_{}_SIZE ({})    /**< Size of {} packet. */\n".format(f._name.upper(), f._total_offset // 8 + 2, f._name.lower())
        else:
            pdu_defines += "#define PACKET_MESH_{}_PDU_OFFSET    ({})    /**< Offset to {} payload. */\n".format(f._name.upper(), f._total_offset // 8, f._name.lower())
            pdu_defines += "#define PACKET_MESH_{}_PDU_MAX_SIZE  ({})    /**< Max PDU size for {} packets. */\n".format(f._name.upper(), 31 - 2 - f._total_offset // 8, f._name.lower())
            pdu_defines += "#define PACKET_MESH_{}_OVERHEAD_SIZE ({})    /**< Header overhead size for {} packets. */\n".format(f._name.upper(), f._total_offset // 8 + 2, f._name.lower())

            pdu_funcs += """/**
 * Gets the {lname} payload pointer.
 * @param[in,out] p_pkt Packet pointer.
 * @returns Pointer to the start of the upper transport PDU.
 */
static inline uint8_t * packet_mesh_{lname}_payload_get(packet_mesh_t * p_pkt)
{{
    return &p_pkt->pdu[PACKET_MESH_{uname}_PDU_OFFSET];
}}

/**
 * Gets the length of the upper transport PDU.
 * @param[in] p_pkt Packet pointer.
 * @returns Length of the upper transport PDU.
 */
static inline uint8_t packet_mesh_{lname}_pdu_length_get(const packet_mesh_t * p_pkt)
{{
    return p_pkt->length - sizeof(p_pkt->ad_type) - PACKET_MESH_{uname}_PDU_OFFSET;
}}

""".format(lname=f._name.lower(), uname=f._name.upper())

    blob = []
    for fmt in fields:
        for f in fmt.get_fields():
            if f not in blob:
                blob += [f]

    print(HEADER_START)
    for f in blob:
        fun = f.get_definitions()
        if fun != "":
            print(fun)

    print(pdu_defines.strip(), "\n")
    print(pdu_funcs.strip())

    for f in blob:
        fun = f.get_functions()
        if fun != "":
            print(fun)

    print(HEADER_END)
