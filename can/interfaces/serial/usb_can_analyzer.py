import struct
import time
import logging

from can import BusABC, Message

import serial
from serial.tools import list_ports

logger = logging.getLogger(__name__)


class UsbCanAnalyzer(BusABC):
    def __init__(self, channel, *args, **kwargs):

        self.channel_info = "USB-CAN Analyzer: " + channel
        self.ser = serial.serial_for_url(channel, baudrate=2000000, bytesize=8, parity="N", stopbits=1, timeout=0.05)

        super().__init__(channel=channel, *args, **kwargs)

    def shutdown(self):
        self.ser.close()

    def flush_tx_buffer(self):
        self.ser.reset_output_buffer()

    def send(self, msg, timeout=None):
        byte_msg = bytearray()
        byte_msg.append(0xAA)
        byte_msg.append(0xC0 | (msg.is_extended_id << 5) | (msg.is_remote_frame << 4) | msg.dlc)
        byte_msg += struct.pack("<I" if msg.is_extended_id else "<H", msg.arbitration_id)
        byte_msg += msg.data
        byte_msg.append(0x55)
        self.ser.write(byte_msg)
        self.ser.flush()

    def _recv_internal(self, timeout):
        try:
            rx_byte = self.ser.read()
        except serial.SerialException:
            return None, False

        if rx_byte and ord(rx_byte) == 0xAA:
            tyep = ord(self.ser.read(1))
            is_extended_id = tyep & 0x20
            is_remote_frame = tyep & 0x10
            dlc = (tyep << 4 & 0xFF) >> 4
            arb_id = (
                struct.unpack("<I" if is_extended_id else "<H", bytearray(self.ser.read(4 if is_extended_id else 2)))
            )[0]
            data = self.ser.read(dlc)
            rxd_byte = ord(self.ser.read(1))
            if rxd_byte == 0x55:
                msg = Message(
                    arbitration_id=arb_id,
                    timestamp=time.perf_counter(),
                    dlc=dlc,
                    data=data,
                    is_extended_id=is_extended_id,
                    is_remote_frame=is_remote_frame,
                )
                return msg, False

        else:
            return None, False

    def fileno(self):
        if hasattr(self.ser, "fileno"):
            return self.ser.fileno()
        return -1

    @staticmethod
    def _detect_available_configs():
        channels = []
        serial_ports = []

        if list_ports:
            serial_ports = list_ports.comports()

        for port in serial_ports:
            channels.append({"interface": "serial", "channel": port.device})
        return channels
