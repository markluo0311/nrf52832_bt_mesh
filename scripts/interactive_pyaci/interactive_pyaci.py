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

import logging
import IPython
import DateTime
import sys
import os
import colorama

from argparse import ArgumentParser
from traitlets import config

from aci.aci_uart import Uart
from aci.aci_utils import STATUS_CODE_LUT
from aci.aci_config import ApplicationConfig
import aci.aci_cmd as cmd
import aci.aci_evt as evt

from provisioning import Provisioner, Provisionee  # NOQA: ignore unused import


USAGE_STRING = \
    """
    {c_default}{c_text}To control your device use {c_highlight}d[x]{c_text}, type {c_highlight}d[x].{c_text} and hit tab to see the available methods.
    x is for the device index; devices will be indexed based on the order of com ports
    given with the option -d (the first device, {c_highlight}d[0]{c_text}, can also be accessed using {c_highlight}device{c_text}).
""" # NOQA: Ignore long line
USAGE_STRING += colorama.Style.RESET_ALL

FILE_LOG_FORMAT = "%(asctime)s - %(levelname)s - %(name)s: %(message)s"
STREAM_LOG_FORMAT = "%(asctime)s - %(levelname)s - %(name)s: %(message)s"
COLOR_LIST = [colorama.Fore.MAGENTA, colorama.Fore.CYAN,
              colorama.Fore.GREEN, colorama.Fore.YELLOW,
              colorama.Fore.BLUE, colorama.Fore.RED]
COLOR_INDEX = 0


def configure_logger(device_name):
    global options
    global COLOR_INDEX

    logger = logging.getLogger(device_name)
    logger.setLevel(logging.DEBUG)

    stream_formatter = logging.Formatter(
        COLOR_LIST[COLOR_INDEX % len(COLOR_LIST)] + colorama.Style.BRIGHT
        + STREAM_LOG_FORMAT
        + colorama.Style.RESET_ALL)
    COLOR_INDEX = (COLOR_INDEX + 1) % len(COLOR_LIST)

    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(stream_formatter)
    stream_handler.setLevel(options.log_level)
    logger.addHandler(stream_handler)

    if not options.no_logfile:
        dt = DateTime.DateTime()
        logfile = "{}_{}-{}-{}-{}.outlog".format(
            device_name, dt.yy(), dt.dayOfYear(), dt.hour(), dt.minute())
        fh = logging.FileHandler(logfile)
        fh.setLevel(logging.DEBUG)
        file_formatter = logging.Formatter(FILE_LOG_FORMAT)
        fh.setFormatter(file_formatter)
        logger.addHandler(fh)
    return logger


class Interactive(object):
    DEFAULT_APP_KEY = bytearray([0xAA] * 16)
    DEFAULT_SUBNET_KEY = bytearray([0xBB] * 16)
    DEFAULT_VIRTUAL_ADDRESS = bytearray([0xCC] * 16)
    DEFAULT_STATIC_AUTH_DATA = bytearray([0xDD] * 16)
    DEFAULT_LOCAL_UNICAST_ADRESS_START = 0x0001
    CONFIG = ApplicationConfig(
        header_path=os.path.join(os.path.dirname(sys.argv[0]),
                                 ("../../examples/serial/include/"
                                  + "nrf_mesh_config_app.h")))
    PRINT_ALL_EVENTS = True

    def __init__(self, acidev):
        self.acidev = acidev
        self.acidev.add_packet_recipient(self.__event_handler)
        self.logger = configure_logger(self.acidev.device_name)
        self.send = self.acidev.write_aci_cmd

        self._event_filter = []
        self._event_filter_enabled = True
        self._other_events = []

        # Increment the local unicast address range
        # for the next Interactive instance
        self.local_unicast_adress_start = (
            self.DEFAULT_LOCAL_UNICAST_ADRESS_START)
        Interactive.DEFAULT_LOCAL_UNICAST_ADRESS_START += (
            self.CONFIG.ACCESS_ELEMENT_COUNT)

    def close(self):
        self.acidev.stop()

    def events_get(self):
        return self._other_events

    def event_filter_add(self, event_filter):
        self._event_filter += event_filter

    def event_filter_disable(self):
        self._event_filter_enabled = False

    def event_filter_enable(self):
        self._event_filter_enabled = True

    def device_port_get(self):
        return self.acidev.serial.port

    def quick_setup(self):
        self.send(cmd.SubnetAdd(0, bytearray(self.DEFAULT_SUBNET_KEY)))
        self.send(cmd.AppkeyAdd(0, 0, bytearray(self.DEFAULT_APP_KEY)))
        self.send(cmd.AddrLocalUnicastSet(
            self.local_unicast_adress_start,
            self.CONFIG.ACCESS_ELEMENT_COUNT))

    def __event_handler(self, event):
        if self._event_filter_enabled and event._opcode in self._event_filter:
            # Ignore event
            return
        if event._opcode == evt.Event.DEVICE_STARTED:
            self.logger.info("Device rebooted.")

        elif event._opcode == evt.Event.CMD_RSP:
            if event._data["status"] != 0:
                self.logger.error("{}: {}".format(
                    cmd.response_deserialize(event),
                    STATUS_CODE_LUT[event._data["status"]]["code"]))
            else:
                text = str(cmd.response_deserialize(event))
                if text == "None":
                    text = "Success"
                self.logger.info(text)
        else:
            if self.PRINT_ALL_EVENTS and event is not None:
                self.logger.info(str(event))
            else:
                self._other_events.append(event)


def start_ipython(options):
    colorama.init()
    comports = options.devices
    d = list()

    # Print out a mini intro to the interactive session --
    # Start with white and then magenta to keep the session white
    # (workaround for a bug in ipython)
    colors = {"c_default": colorama.Fore.WHITE + colorama.Style.BRIGHT,
              "c_highlight": colorama.Fore.YELLOW + colorama.Style.BRIGHT,
              "c_text": colorama.Fore.CYAN + colorama.Style.BRIGHT}

    print(USAGE_STRING.format(**colors))

    for dev_com in comports:
        d.append(Interactive(Uart(port=dev_com,
                                  baudrate=options.baudrate,
                                  device_name=dev_com.split("/")[-1])))

    device = d[0]
    send = device.acidev.write_aci_cmd  # NOQA: Ignore unused variable

    # Set iPython configuration
    c = config.get_config()
    if options.no_logfile:
        c.TerminalInteractiveShell.logstart = False
        c.InteractiveShellApp.db_log_output = False
    else:
        dt = DateTime.DateTime()
        logfile = "interactive_session_{}-{}-{}-{}.log".format(
            dt.yy(), dt.dayOfYear(), dt.hour(), dt.minute())

        c.TerminalInteractiveShell.logstart = True
        c.InteractiveShellApp.db_log_output = True
        c.TerminalInteractiveShell.logfile = logfile

    c.TerminalInteractiveShell.confirm_exit = False
    c.InteractiveShellApp.multiline_history = True
    c.InteractiveShellApp.log_level = logging.DEBUG

    IPython.embed(config=c)
    for dev in d:
        dev.close()
    raise SystemExit(0)


if __name__ == '__main__':
    parser = ArgumentParser(
        description="nRF5 SDK for Bluetooth Mesh Interactive PyACI")
    parser.add_argument("-d", "--device",
                        dest="devices",
                        nargs="+",
                        required=True,
                        help=("Device Communication port, e.g., COM216 or "
                              + "/dev/ttyACM0. You may connect to multiple "
                              + "devices. Separate devices by spaces, e.g., "
                              + "\"-d COM123 COM234\""))
    parser.add_argument("-b", "--baudrate",
                        dest="baudrate",
                        required=False,
                        default='115200',
                        help="Baud rate. Default: 115200")
    parser.add_argument("--no-logfile",
                        dest="no_logfile",
                        action="store_true",
                        required=False,
                        default=False,
                        help="Disables logging to file.")
    parser.add_argument("-l", "--log-level",
                        dest="log_level",
                        type=int,
                        required=False,
                        default=3,
                        help=("Set default logging level: "
                              + "1=Errors only, 2=Warnings, 3=Info, 4=Debug"))
    options = parser.parse_args()

    if options.log_level == 1:
        options.log_level = logging.ERROR
    elif options.log_level == 2:
        options.log_level = logging.WARNING
    elif options.log_level == 3:
        options.log_level = logging.INFO
    else:
        options.log_level = logging.DEBUG

    start_ipython(options)
