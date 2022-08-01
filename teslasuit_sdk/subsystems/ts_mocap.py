from ..ts_types import TsDeviceHandle
from ctypes import (c_void_p, c_uint8,
                    c_uint64, c_float,
                    pointer, POINTER,
                    Structure, CFUNCTYPE,
                    cast, c_uint)

from ..ts_mapper import TsBone2dIndex

import time


class TsVec2f(Structure):
    _pack_ = 1
    _fields_ = [('x', c_float),
                ('y', c_float)]

    def __str__(self):
        return f'{self.x};{self.y}'


class TsVec3f(Structure):
    _pack_ = 1
    _fields_ = [('x', c_float),
                ('y', c_float),
                ('z', c_float)]

    def __str__(self):
        return f'{self.x:.2f};{self.y:.2f};{self.z:.2f}'


class TsQuat(Structure):
    _pack_ = 1
    _fields_ = [('w', c_float),
                ('x', c_float),
                ('y', c_float),
                ('z', c_float)]

    def __str__(self):
        return f'{self.w:.4f};{self.x:.4f};{self.y:.4f};{self.z:.4f}'


class TsMocapBone(Structure):
    _pack_ = 1
    _fields_ = [('position', TsVec3f),
                ('rotation', TsQuat)]

    def __str__(self):
        return (
            f'position={self.position}, rotation={self.rotation}')


class TsMocapSensor(Structure):
    _pack_ = 1
    _fields_ = [('q6', TsQuat),
                ('q9', TsQuat),
                ('accel', TsVec3f),
                ('gyro', TsVec3f),
                ('magn', TsVec3f),
                ('linear_accel', TsVec3f),
                ('timestamp', c_uint64)]

    def __str__(self):
        return (
            f'q6={self.q6}, q9={self.q9},'
            f'gyro={self.gyro}, magn={self.magn}, accel={self.accel},'
            f'linear_accel={self.linear_accel}, timestamp={self.timestamp}\n')


WAIT_FOR_DATA_READY_TIMEOUT = 0.001


class TsMocap:
    def __init__(self, lib, device_handle):
        self.__lib = lib
        self.__device_handle = device_handle

        self.__is_data_raw_ready = False
        self.__is_data_skeleton_ready = False
        self.__is_started = False

        self.__data_raw_callback = None
        self.__data_skeleton_callback = None

        self.__data_raw = {}
        self.__data_skeleton = {}

    def start_streaming(self):
        if self.__is_started:
            return

        self.__subscribe_on_raw_data_update()
        self.__subscribe_on_skeleton_data_updated()
        self.__lib.ts_mocap_start_streaming(self.__device_handle)

        self.__is_started = True

    def stop_streaming(self):
        if not self.__is_started:
            return

        self.__lib.ts_mocap_stop_streaming(pointer(self.__device_handle))
        self.__is_started = False
    
    def calibrate_skeleton(self):
        self.__lib.ts_mocap_skeleton_calibrate(self.__device)
    
    def get_raw_data_on_ready(self):
        while not self.__is_data_raw_ready and self.__is_started:
            time.sleep(WAIT_FOR_DATA_READY_TIMEOUT)

        self.__is_data_raw_ready = False
        return self.__data_raw

    def get_skeleton_data_on_ready(self):
        while not self.__is_data_skeleton_ready and self.__is_started:
            time.sleep(WAIT_FOR_DATA_READY_TIMEOUT)

        self.__is_data_skeleton_ready = False
        return self.__data_skeleton

    def __subscribe_on_raw_data_update(self):
        def __on_raw_updated_callback(device_handle, data_ptr, user_data):

            for bone_index in list(TsBone2dIndex):
                sensor_data = self.get_sensor_data(bone_index, data_ptr)
                self.__data_raw[bone_index] = sensor_data

            self.__is_data_raw_ready = True

        raw_data_callback_prototype = CFUNCTYPE(None, POINTER(TsDeviceHandle), c_void_p, c_void_p)
        self.__data_raw_callback = raw_data_callback_prototype(__on_raw_updated_callback)
        self.__lib.ts_mocap_set_sensor_skeleton_update_callback(self.__device_handle, self.__data_raw_callback,
                                                                c_void_p())

    def __subscribe_on_skeleton_data_updated(self):
        def __on_skeleton_updated_callback(device_ptr, data_ptr, user_data):
            for bone_index in list(TsBone2dIndex):
                bone_data = self.get_skeleton_data(bone_index, data_ptr)
                self.__data_skeleton[bone_index] = bone_data

            self.__is_data_skeleton_ready = True

        skeleton_data_callback_prototype = CFUNCTYPE(None, POINTER(TsDeviceHandle), c_void_p, c_void_p)
        self.__data_skeleton_callback = skeleton_data_callback_prototype(__on_skeleton_updated_callback)
        self.__lib.ts_mocap_set_skeleton_update_callback(self.__device_handle, self.__data_skeleton_callback,
                                                         c_void_p())

    def get_sensor_data(self, bone_index, data_ptr):
        bone = TsMocapSensor()
        self.__lib.ts_mocap_sensor_skeleton_get_bone(c_void_p(data_ptr), c_uint(bone_index), pointer(bone))

        return bone

    def get_skeleton_data(self, bone_index, data_ptr):
        bone = TsMocapBone()
        self.__lib.ts_mocap_skeleton_get_bone(c_void_p(data_ptr), bone_index, pointer(bone))

        return bone
