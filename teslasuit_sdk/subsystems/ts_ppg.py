from ctypes import (c_void_p, c_uint8,
                    c_uint32, c_uint64,
                    c_bool, CFUNCTYPE,
                    POINTER, pointer,
                    Structure, cast,
                    c_float)
from teslasuit_sdk.ts_types import TsDeviceHandle
from enum import Enum
import time


class PpgSensorType(Enum):
    UNDEFINED = 0
    MAX30102 = 1
    MAX86916 = 2

class TsHrv(Structure):
    _pack_ = 1
    _fields_ = [('mean_rr', c_float),
                ('sdnn', c_float),
                ('sdsd', c_float),
                ('rmssd', c_float),
                ('sd1', c_float),
                ('sd2', c_float),
                ('hlf', c_float)]

class TsPpgNodeData(Structure):
    _pack_ = 1
    _fields_ = [('heart_rate', c_uint32),
                ('oxygen_percent', c_uint8),
                ('is_heart_rate_valid', c_bool),
                ('is_oxygen_percent_valid', c_bool),
                ('timestamp', c_uint32)]


class TsPpgData(Structure):
    _pack_ = 1
    _fields_ = [('number_of_nodes', c_uint32),
                ('nodes', POINTER(TsPpgNodeData))]

    def __str__(self):
        assert self.nodes is not None
        return str("PPG nodes number = {}, first frame heart rate = {}".format(self.number_of_nodes,
                                                                               self.nodes[0].is_heart_rate_valid))


class TsPpgRawNodeData(Structure):
    _pack_ = 1
    _fields_ = [('sample_size', c_uint64),
                ('ir_data', POINTER(c_uint64)),
                ('red_data', POINTER(c_uint64)),
                ('blue_data', POINTER(c_uint64)),
                ('green_data', POINTER(c_uint64)),
                ('timestamp', c_uint64)]


class TsPpgRawData(Structure):
    _pack_ = 1
    _fields_ = [('number_of_nodes', c_uint8),
                ('nodes', POINTER(TsPpgRawNodeData))]

    def __str__(self):
        assert self.nodes is not None
        return f'Raw PPG nodes number = {self.number_of_nodes}, first frame sample = {self.nodes[0].sample_size}'


class TsPpg:
    def __init__(self, lib, device):
        self.__lib = lib
        self.__device = device

        self.__is_hrv_data_ready = False
        self.__is_data_ready = False
        self.__is_data_raw_ready = False

        self.__is_started = False
        self.__is_started_raw = False

        self.__hrv_callback = None
        self.__data_callback = None
        self.__data_raw_callback = None

        self.__hrv = TsHrv()
        self.__data = TsPpgData()
        self.__data_raw = TsPpgRawData()

    def start_raw_streaming(self):
        if self.__is_started_raw:
            return

        self.__subscribe_on_data_update()
        self.__subscribe_on_data_raw_update()

        self.__lib.ts_ppg_raw_start_streaming(self.__device)
        self.__is_started_raw = True

    def stop_raw_streaming(self):
        if not self.__is_started_raw:
            return

        self.__lib.ts_ppg_raw_stop_streaming(self.__device)
        self.__is_started_raw = False

    def get_hrv_data_on_ready(self):
        while not self.__is_hrv_data_ready and self.__is_started_raw:
            time.sleep(0.001)

        self.__is_hrv_data_ready = False
        return self.__hrv

    def get_raw_data_on_ready(self):
        while not self.__is_data_raw_ready and self.__is_started_raw:
            time.sleep(0.001)

        self.__is_data_raw_ready = False
        return self.__data_raw

    def get_hrv(self):
        return self.__hrv

    def get_data(self):
        return self.__data

    def get_data_raw(self):
        return self.__data_raw

    def calibrate(self):
        self.__lib.ts_ppg_calibrate(self.__device)

    def __subscribe_on_data_raw_update(self):
        def __on_hrv_updated(device_ptr, data_ptr, user_data):
            self.__lib.ts_hrv_get_data(c_void_p(data_ptr), pointer(self.__hrv))
            self.__is_hrv_data_ready = True
            
        hrv_data_callback_prototype = CFUNCTYPE(None, POINTER(TsDeviceHandle), c_void_p, c_void_p)
        self.__hrv_callback = hrv_data_callback_prototype(__on_hrv_updated)
        self.__lib.ts_hrv_set_update_callback(self.__device, self.__hrv_callback,
                                                  c_void_p(0))
                                                  
        def __on_updated_raw_callback(device_ptr, data_ptr, user_data):
            self.__parse_data_raw(data_ptr)
            self.__is_data_raw_ready = True

        data_raw_callback_prototype = CFUNCTYPE(None, POINTER(TsDeviceHandle), c_void_p, c_void_p)
        self.__data_raw_callback = data_raw_callback_prototype(__on_updated_raw_callback)
        self.__lib.ts_ppg_raw_set_update_callback(self.__device, self.__data_raw_callback,
                                                  c_void_p(0))

    def __subscribe_on_data_update(self):
        def __on_updated_callback(device_ptr, data_ptr, user_data):
            nodes_indexes = self.__get_nodes_indexes(data_ptr)
            self.__data.number_of_nodes = len(nodes_indexes)

            data = (TsPpgNodeData * len(nodes_indexes))()

            for i, node_index in enumerate(nodes_indexes):
                heart_rate = self.__get_heart_rate(data_ptr, node_index)
                oxygen_percent = self.__get_oxygen_percent(data_ptr, node_index)
                is_valid_heart_rate = self.__get_is_valid_heart_rate(data_ptr, node_index)
                is_valid_oxygen_percent = self.__get_is_valid_oxygen_percent(data_ptr, node_index)
                timestamp = self.__get_timestamp(data_ptr, node_index)

                data[i] = TsPpgNodeData(heart_rate, oxygen_percent,
                                        is_valid_heart_rate, is_valid_oxygen_percent,
                                        timestamp)

            self.__data.nodes = cast(data, POINTER(TsPpgNodeData))
            self.__is_data_ready = True

        data_callback_prototype = CFUNCTYPE(None, POINTER(TsDeviceHandle), c_void_p, c_void_p)
        self.__data_callback = data_callback_prototype(__on_updated_callback)
        self.__lib.ts_ppg_set_update_callback(self.__device, self.__data_callback,
                                              c_void_p(0))

    def __parse_data_raw(self, data_ptr):
        nodes_indexes = self.__get_nodes_indexes(data_ptr)
        data = (TsPpgRawNodeData * len(nodes_indexes))()

        for i, node_index in enumerate(nodes_indexes):
            sample_size = self.__get_raw_number_of_sample(data_ptr, node_index)

            ir_data = self.__get_raw_ir_data(data_ptr, node_index)
            red_data = self.__get_raw_red_data(data_ptr, node_index)
            green_data = self.__get_raw_green_data(data_ptr, node_index)
            blue_data = self.__get_raw_blue_data(data_ptr, node_index)
            timestamp = self.__get_timestamp(data_ptr, node_index)

            data[i] = TsPpgRawNodeData(sample_size,
                                       cast(ir_data, POINTER(c_uint64)), cast(red_data, POINTER(c_uint64)),
                                       cast(blue_data, POINTER(c_uint64)), cast(green_data, POINTER(c_uint64)),
                                       timestamp)

        self.__data_raw.number_of_nodes = len(nodes_indexes)
        self.__data_raw.nodes = cast(data, POINTER(TsPpgRawNodeData))

    def __get_number_of_nodes(self, data_ptr):
        number_of_nodes = c_uint8(0)
        self.__lib.ts_ppg_get_number_of_nodes(c_void_p(data_ptr), pointer(number_of_nodes))

        return number_of_nodes.value

    def __get_nodes_indexes(self, data_ptr):
        number_of_nodes = self.__get_number_of_nodes(data_ptr)
        nodes_indexes = (c_uint8 * number_of_nodes)()

        self.__lib.ts_ppg_get_node_indexes(c_void_p(data_ptr), pointer(nodes_indexes), number_of_nodes)

        return [*nodes_indexes]

    def __get_heart_rate(self, data_ptr, node_index):
        heart_rate = c_uint8(0)
        self.__lib.ts_ppg_get_heart_rate(c_void_p(data_ptr), c_uint8(node_index), pointer(heart_rate))

        return heart_rate.value

    def __get_oxygen_percent(self, data_ptr, node_index):
        oxygen_percent = c_uint8(0)
        self.__lib.ts_ppg_get_oxygen_percent(c_void_p(data_ptr), c_uint8(node_index), pointer(oxygen_percent))

        return oxygen_percent.value

    def __get_is_valid_heart_rate(self, data_ptr, node_index):
        is_valid = c_uint8(0)
        self.__lib.ts_ppg_is_heart_rate_valid(c_void_p(data_ptr), c_uint8(node_index), pointer(is_valid))

        return is_valid.value

    def __get_is_valid_oxygen_percent(self, data_ptr, node_index):
        is_valid = c_uint8(0)
        self.__lib.ts_ppg_is_oxygen_percent_valid(c_void_p(data_ptr), c_uint8(node_index), pointer(is_valid))

        return is_valid.value

    def __get_timestamp(self, data_ptr, node_index):
        timestamp = c_uint64(0)
        self.__lib.ts_ppg_get_timestamp(c_void_p(data_ptr), c_uint8(node_index), pointer(timestamp))

        return timestamp.value

    def __get_raw_number_of_sample(self, data_ptr, node_index):
        number_of_samples = c_uint64(0)
        self.__lib.ts_ppg_raw_get_data_size(c_void_p(data_ptr), node_index, pointer(number_of_samples))

        return number_of_samples.value

    def __get_raw_red_data(self, data_ptr, node_index):
        number_of_samples = self.__get_raw_number_of_sample(data_ptr, node_index)

        red_data = (c_uint64 * number_of_samples)()
        self.__lib.ts_ppg_raw_get_red_data(c_void_p(data_ptr), c_uint8(node_index), pointer(red_data))

        return red_data

    def __get_raw_ir_data(self, data_ptr, node_index):
        number_of_samples = self.__get_raw_number_of_sample(data_ptr, node_index)

        ir_data = (c_uint64 * number_of_samples)()
        self.__lib.ts_ppg_raw_get_infrared_data(c_void_p(data_ptr), c_uint8(node_index), pointer(ir_data))

        return ir_data

    def __get_raw_blue_data(self, data_ptr, node_index):
        number_of_samples = self.__get_raw_number_of_sample(data_ptr, node_index)

        blue_data = (c_uint64 * number_of_samples)()
        self.__lib.ts_ppg_raw_get_blue_data(c_void_p(data_ptr), c_uint8(node_index), pointer(blue_data))

        return blue_data

    def __get_raw_green_data(self, data_ptr, node_index):
        number_of_samples = self.__get_raw_number_of_sample(data_ptr, node_index)

        green_data = (c_uint64 * number_of_samples)()
        self.__lib.ts_ppg_raw_get_green_data(c_void_p(data_ptr), c_uint8(node_index), pointer(green_data))

        return green_data
