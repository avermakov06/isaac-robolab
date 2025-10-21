import struct
from threading import main_thread
from dataclasses import fields
from API.source.models.constants import RTD_PORT

from API.source.models.classes.data_classes.rtd_structure import (
    RTD, RTDataPackageBorders, STRUCT_FORMAT
)
from API.source.core.network.socket_factory import SocketWrapper
from API.source.features.tools import sleep
from API.source.core.exceptions.data_validation_error.parsing_error import (
    RTDParsingError
)
from API.source.models.constants import CHECK_FREQUENCY_SEC


class RTDReceiver(SocketWrapper):
    def __init__(self, ip: str, timeout: int):
        SocketWrapper.__init__(self, ip, RTD_PORT, timeout)
        self.finishing_receiving = False
        self.rt_data = RTD()
        self._struct_size = struct.calcsize(STRUCT_FORMAT)
        self._receiving_in_process = False

    def _receive_rtd(self) -> bool:
        raw_data = self.recv_(self._struct_size)
        while len(raw_data) < self._struct_size:
            chunk = self.recv_(self._struct_size - len(raw_data))
            raw_data += chunk
        unpack_data = struct.unpack(STRUCT_FORMAT, raw_data)
        n = 0
        for field in fields(RTD):
            if (amount := getattr(RTD, field.name)) > 1:
                setattr(self.rt_data, field.name, unpack_data[n:n + amount])
            else:
                setattr(self.rt_data, field.name, unpack_data[n])
            n += amount
        return True

    def receiving_loop(self):
        try:
            while self.get_active() and main_thread().is_alive():
                self._receiving_in_process = True
                self._receive_rtd()
                self._receiving_in_process = False
        except Exception as e:
            self._receiving_in_process = False
            raise e

    def shutdown(self) -> None:
        self.set_active(False)
        for _ in sleep(await_sec=self.timeout, frequency=CHECK_FREQUENCY_SEC):
            if not self._receiving_in_process:
                break
        super().shutdown()

    def _validate_rtd(self) -> bool:
        if (
            self.rt_data.packet_begin == RTDataPackageBorders.rtd_packet_begin
            and self.rt_data.packet_end == RTDataPackageBorders.rtd_packet_end
        ):
            return True
        raise RTDParsingError('Most probably incorrect byte parsing')

    def initialise(self) -> bool:
        return bool(
            self.create_connection() and self._receive_rtd()
            and self._validate_rtd()
        )
