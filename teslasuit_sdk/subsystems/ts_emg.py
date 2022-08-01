from ctypes import (c_void_p, c_uint8,
                    c_uint32, c_uint64,
                    c_int64, CFUNCTYPE,
                    POINTER, pointer,
                    Structure, cast)
from teslasuit_sdk.ts_types import TsDeviceHandle
import time


class TsEmgOptions(Structure):
    _pack_ = 1
    _fields_ = [('lower_bandwidth', c_uint32),
                ('upper_bandwidth', c_uint32),
                ('sampling_frequency', c_uint32),
                ('sample_size', c_uint8)]

    def __str__(self):
        return f'sample_size={self.sample_size}, sampling_frequency={self.sampling_frequency}, '\
               f'lower_bandwidth={self.lower_bandwidth}, upper_bandwidth={self.upper_bandwidth}'


class TsEmgChannelData(Structure):
    _pack_ = 1
    _fields_ = [('channel_index', c_uint8),
                ('number_of_samples', c_uint64),
                ('samples', POINTER(c_int64))]

    def __str__(self):
        data = 'channel_index = {0}\n' \
               'number_of_samples = {1}\nsamples = ['.format(self.channel_index, self.number_of_samples)

        for i in range(0, self.number_of_samples):
            data += str(self.samples[i]) + ','

        return data + ']'


class TsEmgNodeData(Structure):
    _pack_ = 1
    _fields_ = [('node_index', c_uint8),
                ('number_of_channels', c_uint8),
                ('channels', POINTER(TsEmgChannelData)),
                ('number_of_timestamps', c_uint64),
                ('timestamps', POINTER(c_uint64))]

    def __str__(self):
        data = 'node_index = {0},\n' \
               'number_of_channels = {1}\n'.format(self.node_index, self.number_of_channels)

        for i in range(0, self.number_of_channels):
            data += '[\n' + str(self.channels[i]) + '\n]\n'

        return data


class TsEmgData(Structure):
    _pack_ = 1
    _fields_ = [('number_of_nodes', c_uint8),
                ('nodes', POINTER(TsEmgNodeData)),
                ('options', POINTER(TsEmgOptions))]

    def __str__(self):
        data = 'number_of_nodes = {0}\n'.format(self.number_of_nodes)

        for i in range(0, self.number_of_nodes):
            data += '{\n' + str(self.nodes[i]) + '\n}'
        return data


class TsEmg:
    def __init__(self, lib, device):
        self.__lib = lib
        self.__device = device

        self.__is_data_ready = False
        self.__is_started = False

        self.__data_callback = None

        self.__data = TsEmgData()

    def set_options(self, lower_bandwidth, upper_bandwidth,
                    sample_frequency, sample_size):
        emg_options = TsEmgOptions(lower_bandwidth, upper_bandwidth,
                                   sample_frequency, sample_size)
        self.__lib.ts_emg_set_options(self.__device, emg_options)

    def start_streaming(self):
        if self.__is_started:
            return

        self.__subscribe_on_data_update()

        self.__lib.ts_emg_start_streaming(self.__device)
        self.__is_started = True

    def stop_streaming(self):
        if not self.__is_started:
            return

        self.__lib.ts_emg_stop_streaming(self.__device)
        self.__is_started = False

    def get_data(self):
        self.__is_data_ready = False
        return self.__data

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
            self.__is_data_ready = True

        data_callback_prototype = CFUNCTYPE(None, POINTER(TsDeviceHandle), c_void_p, c_void_p)
        self.__data_callback = data_callback_prototype(__on_updated_callback)
        self.__lib.ts_emg_set_update_callback(self.__device, self.__data_callback,
                                              c_void_p(0))

    def __parse_data(self, data_ptr):
        emg_options = self.__get_options(data_ptr)
        self.__data.options = pointer(emg_options)

        nodes_indexes = self.__get_nodes_indexes(data_ptr)
        self.__data.number_of_nodes = len(nodes_indexes)

        nodes_data = (TsEmgNodeData * len(nodes_indexes))()

        for i, node_index in enumerate(nodes_indexes):
            number_of_channels = self.__get_number_of_channels(data_ptr, node_index)

            nodes_data[i].node_index = node_index
            nodes_data[i].number_of_channels = number_of_channels

            channels = (TsEmgChannelData * number_of_channels)()

            for channel_index in range(0, number_of_channels):
                number_of_samples = self.__get_channel_sample_size(data_ptr, node_index)
                samples = self.__get_channel_sample(data_ptr, node_index, channel_index)

                channels[channel_index].channel_index = channel_index
                channels[channel_index].number_of_samples = number_of_samples
                channels[channel_index].samples = cast(samples, POINTER(c_int64))

            nodes_data[i].channels = cast(channels, POINTER(TsEmgChannelData))
            timestamps = self.__get_timestamps(data_ptr, node_index)

            nodes_data[i].number_of_timestamps = len(timestamps)
            nodes_data[i].timestamps = cast(timestamps, POINTER(c_uint64))

        self.__data.nodes = cast(nodes_data, POINTER(TsEmgNodeData))

    def __get_options(self, data_ptr):
        emg_options = TsEmgOptions()
        self.__lib.ts_emg_get_options(c_void_p(data_ptr), pointer(emg_options))

        return emg_options

    def __get_number_of_nodes(self, data_ptr):
        number_of_nodes = c_uint8(0)
        self.__lib.ts_emg_get_number_of_nodes(c_void_p(data_ptr), pointer(number_of_nodes))

        return number_of_nodes.value

    def __get_nodes_indexes(self, data_ptr):
        number_of_nodes = self.__get_number_of_nodes(data_ptr)
        nodes_indexes = (c_uint8 * number_of_nodes)()
        self.__lib.ts_emg_get_node_indexes(c_void_p(data_ptr), pointer(nodes_indexes), number_of_nodes)

        return [*nodes_indexes]

    def __get_number_of_channels(self, data_ptr, node_index):
        number_of_channels = c_uint8(0)
        self.__lib.ts_emg_get_number_of_channels(c_void_p(data_ptr), c_uint8(node_index),
                                                        pointer(number_of_channels))
        return number_of_channels.value

    def __get_channel_sample_size(self, data_ptr, node_index):
        sample_size = c_uint64(0)
        self.__lib.ts_emg_get_channel_data_size(c_void_p(data_ptr), node_index,
                                                        pointer(sample_size))

        return sample_size.value

    def __get_channel_sample(self, data_ptr, node_index, channel_index):
        number_of_samples = self.__get_channel_sample_size(data_ptr, node_index)
        samples = (c_int64 * number_of_samples)()

        self.__lib.ts_emg_get_channel_data(c_void_p(data_ptr), c_uint8(node_index), c_uint32(channel_index),
                                           pointer(samples), c_uint64(number_of_samples))
        return samples

    def __get_number_of_timestamps(self, data_ptr, node_index):
        number_of_timestamps = c_uint64(0)
        self.__lib.ts_emg_get_number_of_node_timestamps(c_void_p(data_ptr), c_uint8(node_index),
                                                        pointer(number_of_timestamps))
        return number_of_timestamps.value

    def __get_timestamps(self, data_ptr, node_index):
        number_of_timestamps = self.__get_number_of_timestamps(data_ptr, node_index)
        timestamps = (c_uint64 * number_of_timestamps)()

        self.__lib.ts_emg_get_node_timestamps(c_void_p(data_ptr), c_uint8(node_index),
                                              pointer(timestamps), c_uint64(number_of_timestamps))
        return timestamps
