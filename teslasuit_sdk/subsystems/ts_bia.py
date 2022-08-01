from ctypes import (c_uint8, POINTER,
                    pointer, Structure,
                    c_uint64, c_uint32,
                    CFUNCTYPE, c_void_p, cast)
from teslasuit_sdk.ts_types import TsDeviceHandle
import time


class TsComplexNumber(Structure):
    _pack_ = 1
    _fields_ = [('real_value', c_uint32),
                ('im_value', c_uint32)]


class TsBiaFrequencyData(Structure):
    _pack_ = 1
    _fields_ = [('frequency', c_uint32),
                ('complex_number', TsComplexNumber)]


class TsBiaChannelData(Structure):
    _pack_ = 1
    _fields_ = [('number_of_frequencies', c_uint32),
                ('frequencies', POINTER(TsBiaFrequencyData))]


class TsBiaNodeData(Structure):
    _pack_ = 1
    _fields_ = [('number_of_channels', c_uint32),
                ('channels', POINTER(TsBiaChannelData))]


class TsBiaData(Structure):
    _pack_ = 1
    _fields_ = [('number_of_nodes', c_uint8),
                ('nodes', POINTER(TsBiaNodeData))]


class TsBia:
    def __init__(self, lib, device):
        self.__lib = lib
        self.__device = device

        self.__is_started = False
        self.__is_data_ready = False

        self.__data = TsBiaData()

    def set_frequencies(self, start_frequency, stop_frequency, step_frequency):
        self.__lib.ts_bia_set_frequencies(self.__device, start_frequency,
                                          stop_frequency, step_frequency)

    def set_node_channels(self, node_index, channels_indexes):
        channels = (c_uint32 * len(channels_indexes))(*channels_indexes)
        self.__lib.ts_bia_set_node_channels(self.__device, c_uint8(node_index),
                                            pointer(channels), c_uint64(len(channels_indexes)))

    def start_streaming(self):
        if self.__is_started:
            return

        self.__subscribe_on_data_update()

        self.__lib.ts_bia_start_streaming(self.__device)
        self.__is_started = True

    def stop_streaming(self):
        if not self.__is_started:
            return

        self.__lib.ts_bia_stop_streaming(self.__device)
        self.__is_started = False

    def get_data_on_ready(self):
        while not self.__is_data_ready and self.__is_started:
            time.sleep(0.001)

        self.__is_data_ready = False
        return self.__data

    def __subscribe_on_data_update(self):
        def __on_updated_callback(device, data_ptr, user_data):
            nodes_indexes = self.__get_node_indexes(data_ptr)
            self.__data.number_of_nodes = len(nodes_indexes)

            nodes = (TsBiaNodeData * len(nodes_indexes))()

            for i, node_index in enumerate([*nodes_indexes]):
                channels_indexes = self.__get_node_channels_indexes(data_ptr, node_index)
                nodes[i].number_of_channels = len(channels_indexes)

                channels = (TsBiaChannelData * len(channels_indexes))()

                for ii, channel_index in [*channels_indexes]:
                    frequencies_values = self.__get_channel_frequencies(data_ptr, node_index, channel_index)
                    channels[ii].number_of_frequencies = len(frequencies_values)

                    frequencies = (TsBiaFrequencyData * len(frequencies_values))()

                    for iii, frequency in enumerate([*frequencies_values]):
                        complex_number = self.__get_channel_frequency_complex_value(data_ptr, node_index,
                                                                                    channel_index, frequency)
                        frequencies[iii].frequency = frequency
                        frequencies[iii].complex_number = complex_number

                    channels[ii].frequencies = cast(frequencies, POINTER(TsBiaFrequencyData))

                nodes[i].channels = cast(channels, POINTER(TsBiaChannelData))

            self.__data.nodes = cast(nodes, POINTER(TsBiaNodeData))
            self.__is_data_ready = True

        data_callback_prototype = CFUNCTYPE(None, POINTER(TsDeviceHandle), c_void_p, c_void_p)
        self.__data_callback = data_callback_prototype(__on_updated_callback)
        self.__lib.ts_bia_set_update_callback(self.__device, self.__data_callback,
                                              c_void_p())

    def __get_number_of_nodes(self, data_ptr):
        number_of_nodes = c_uint8(0)
        self.__lib.ts_bia_get_number_of_nodes(c_void_p(data_ptr), pointer(number_of_nodes))

        return number_of_nodes.value

    def __get_node_indexes(self, data_ptr):
        number_of_nodes = self.__get_number_of_nodes(data_ptr)
        node_indexes = (c_uint8 * number_of_nodes)()

        self.__lib.ts_bia_get_node_indexes(c_void_p(data_ptr), pointer(node_indexes),
                                           c_uint8(number_of_nodes))
        return node_indexes

    def __get_number_of_channels(self, data_ptr, node_index):
        number_of_channels = c_uint64(0)
        self.__lib.ts_bia_get_number_of_channels(c_void_p(data_ptr), pointer(number_of_channels))

        return number_of_channels.value

    def __get_node_channels_indexes(self, data_ptr, node_index):
        number_of_channels = self.__get_number_of_channels(data_ptr, node_index)
        channel_indexes = (c_uint8 * number_of_channels)()

        self.__lib.ts_bia_get_node_channel_indexes(c_void_p(data_ptr), c_uint8(node_index),
                                                   pointer(channel_indexes), c_uint64(number_of_channels))
        return channel_indexes

    def __get_channel_frequencies_size(self, data_ptr, node_index, channel_index):
        number_of_channel_frequencies = c_uint64(0)
        self.__lib.ts_bia_get_channel_number_of_frequencies(c_void_p(data_ptr), c_uint64(number_of_channel_frequencies))

        return number_of_channel_frequencies.value

    def __get_channel_frequencies(self, data_ptr, node_index, channel_index):
        number_of_channel_frequencies = self.__get_channel_frequencies_size(data_ptr, node_index, channel_index)
        channel_frequencies = (c_uint32 * number_of_channel_frequencies)()

        self.ts_bia_get_channel_frequencies(c_void_p(data_ptr), c_uint8(node_index),
                                            c_uint32(channel_index), pointer(channel_frequencies),
                                            c_uint64(number_of_channel_frequencies))
        return channel_frequencies

    def __get_channel_frequency_complex_value(self, data_ptr,
                                              node_index, channel_index,
                                              frequency):
        complex_number = TsComplexNumber()
        self.__lib.ts_bia_get_channel_frequency_complex_value(c_void_p(data_ptr), c_uint8(node_index),
                                                              c_uint32(channel_index), c_uint32(frequency),
                                                              pointer(complex_number))
        return complex_number
