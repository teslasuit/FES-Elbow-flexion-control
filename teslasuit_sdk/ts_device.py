from teslasuit_sdk import ts_types
from teslasuit_sdk.subsystems import (ts_mocap, ts_emg,
                         ts_ppg, ts_temperature,
                         ts_haptic, ts_bia, ts_magnetic_encoder)

from ctypes import (c_ubyte, pointer,
                    c_void_p, c_char_p,
                    POINTER, c_int)

SUIT_SSID_LENGTH = 32

class TsDevice:
    def __init__(self, lib, device_uuid):
        self.__lib = lib
        self.device_uuid = device_uuid
        self.__device_handle = POINTER(ts_types.TsDeviceHandle)
        self.__open_device(pointer(device_uuid))
        self.__read_properties()

        self.mocap = ts_mocap.TsMocap(lib, self.__device_handle)
        self.emg = ts_emg.TsEmg(lib, self.__device_handle)
        self.ppg = ts_ppg.TsPpg(lib, self.__device_handle)
        self.temperature = ts_temperature.TsTemperature(lib, self.__device_handle)
        self.magnetic_encoder = ts_magnetic_encoder.TsMagneticEncoder(lib, self.__device_handle, self.side)
        self.haptic = ts_haptic.TsHapticPlayer(lib, self.__device_handle)
        self.bia = ts_bia.TsBia(lib, self.__device_handle)

    def __open_device(self, device_uuid):
        ts_device_open = self.__lib.ts_device_open
        ts_device_open.argtypes = [ts_types.TsDevice]
        ts_device_open.restype = POINTER(ts_types.TsDeviceHandle)
        self.__device_handle = ts_device_open(self.device_uuid)

    def __close_device(self):
        self.__lib.ts_device_close(self.__device_handle)

    def __read_properties(self):
        self.type = ts_types.TsDeviceType(self.__lib.ts_device_get_product_type(self.__device_handle))
        if self.type == ts_types.TsDeviceType.Glove:
            self.side = ts_types.TsDeviceSide(self.__lib.ts_device_get_device_side(self.__device_handle))
        else:
            self.side = ts_types.TsDeviceSide.Undefined

    def get_device_ssid(self):
        ssid = (c_ubyte * SUIT_SSID_LENGTH)()
        self.__lib.ts_device_get_name(self.__device_handle, pointer(ssid))
        return list(ssid)

    def get_device_serial(self):
        ts_get_device_serial = self.__lib.ts_device_get_serial
        ts_get_device_serial.argtypes = [POINTER(TsDeviceHandle)]
        ts_get_device_serial.restype = c_char_p
        serial = ts_get_device_serial(self.__device_handle)

    def get_product_type(self):
        ts_device_get_product_type = self.__lib.ts_device_get_product_type
        ts_device_get_product_type.argtypes = [POINTER(ts_types.TsDeviceHandle)]
        ts_device_get_product_type.restype = c_int(0)
        return ts_device_get_product_type(self.__device_handle)

    def get_mapping(self):
        mapping = c_void_p()
        self.__lib.ts_mapping2d_get_by_device(self.__device_handle, pointer(mapping))
        return mapping.value

    def __del__(self):
        self.__close_device()
