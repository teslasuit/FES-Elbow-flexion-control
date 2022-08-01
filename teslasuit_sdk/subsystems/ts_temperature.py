from ctypes import (c_void_p, c_uint8, c_uint16,
                    pointer, c_uint64, Structure,
                    POINTER, CFUNCTYPE, cast)
from teslasuit_sdk.ts_types import TsDeviceHandle
from enum import Enum, unique
import time


@unique
class TsTemperatureSensorType(Enum):
    Undefined = 0
    Si7051 = 1
    TMP117 = 2


class TsTemperatureNodeData(Structure):
    _pack_ = 1
    _fields_ = [('node_index', c_uint8),
                ('sensor_type', c_uint8),
                ('value', c_uint16)]

    def __str__(self):
        return 'node index = {0}, sensor type = {1},' \
               'value = {2}'.format(self.node_index, self.sensor_type, self.value)


class TsTemperatureNodes(Structure):
    _pack_ = 1
    _fields_ = [('number_of_nodes', c_uint8),
                ('node', POINTER(TsTemperatureNodeData)),
                ('timestamp', c_uint64)]

    def __str__(self):
        data = ''
        for i in range(0, self.number_of_nodes):
            data += '{' + str(self.node[i]) + '}'

        return 'number_of_nodes = {0}, nodes =[ {1} ],' \
               ' timestamp = {2}'.format(self.number_of_nodes, data, self.timestamp)


class TsTemperature:
    def __init__(self, lib, device):
        self.__lib = lib
        self.__device = device

        self.__is_started = False
        self.__is_data_ready = False

        self.__data = TsTemperatureNodes()

    def start_streaming(self):
        if self.__is_started:
            return

        self.__subscribe_on_data_update()

        self.__lib.ts_temperature_start_streaming(self.__device)
        self.__is_started = True

    def stop_streaming(self):
        if not self.__is_started:
            return

        self.__lib.ts_temperature_stop_streaming(self.__device)
        self.__is_started = False

    def __str__(self):
        return str(self.__data)

    def get_data_on_ready(self):
        while not self.__is_data_ready and self.__is_started:
            time.sleep(0.001)

        self.__is_data_ready = False
        return self.__data

    def __subscribe_on_data_update(self):
        def __on_updated_callback(device_ptr, data_ptr, user_data):
            self.__parse_data(data_ptr)

        data_callback_prototype = CFUNCTYPE(None, POINTER(TsDeviceHandle), c_void_p, c_void_p)
        self.__data_callback = data_callback_prototype(__on_updated_callback)
        self.__lib.ts_temperature_set_update_callback(self.__device, self.__data_callback,
                                                      c_void_p(0))

    def __parse_data(self, data_ptr):
        nodes_indexes = self.__get_nodes_indexes(data_ptr)
        data = (TsTemperatureNodeData * len(nodes_indexes))()

        for i, node_index in enumerate(nodes_indexes):
            sensor_type = self.__get_sensor_type(data_ptr, node_index)
            value = self.__get_temperature_value(data_ptr, node_index)

            data[i].node_index = c_uint8(node_index)
            data[i].sensor_type = sensor_type
            data[i].value = value

        timestamp = self.__get_timestamp(data_ptr)
        self.__data = TsTemperatureNodes(len(nodes_indexes),
                                         cast(data, POINTER(TsTemperatureNodeData)),
                                         timestamp)

    def __get_number_of_nodes(self, data_ptr):
        number_of_nodes = c_uint8(0)
        self.__lib.ts_temperature_get_number_of_nodes(c_void_p(data_ptr), pointer(number_of_nodes))

        return number_of_nodes.value

    def __get_nodes_indexes(self, data_ptr):
        number_of_nodes = self.__get_number_of_nodes(data_ptr)
        nodes_indexes = (c_uint8 * number_of_nodes)()
        self.__lib.ts_temperature_get_node_indexes(c_void_p(data_ptr), pointer(nodes_indexes), number_of_nodes)

        return [*nodes_indexes]

    def __get_sensor_type(self, data_ptr, node_index):
        sensor_type = c_uint8(0)
        self.__lib.ts_temperature_get_sensor_type(c_void_p(data_ptr), c_uint8(node_index), pointer(sensor_type))

        return sensor_type.value

    def __get_temperature_value(self, data_ptr, node_index):
        temperature_value = c_uint16(0)
        self.__lib.ts_temperature_get_value(c_void_p(data_ptr), c_uint8(node_index), pointer(temperature_value))

        return temperature_value.value

    def __get_timestamp(self, data_ptr):
        timestamp = c_uint64(0)
        self.__lib.ts_temperature_get_timestamp(c_void_p(data_ptr), pointer(timestamp))

        return timestamp.value
