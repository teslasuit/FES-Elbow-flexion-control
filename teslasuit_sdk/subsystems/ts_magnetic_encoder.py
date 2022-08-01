from ctypes import *
from teslasuit_sdk import ts_types
from teslasuit_sdk.ts_mapper import TsBone2dIndex
from enum import Enum, unique, IntEnum


@unique
class TsForceFeedbackLockDirection(Enum):
    Up = 1
    Down = 2
    Both = 3

class TsForceFeedbackConfig(Structure):
    _pack_ = 1
    _fields_ = [('bone_index', c_uint32),
                ('angle', c_float),
                ('hardness_percent', c_uint8),
                ('lock_direction', c_uint32)]

class TsFingerMEPosition:
    def __init__(self):
        self.flexion_angle = 0
        self.abduction_angle = 0

class TsGloveMEPosition:
    def __init__(self, side):
        self.side = side
        if self.side == ts_types.TsDeviceSide.Left:
            self.fingers = get_left_default_position_struct()
        elif self.side == ts_types.TsDeviceSide.Right:
            self.fingers = get_right_default_position_struct()
        else:
            self.fingers = dict()

ENCODER_COUNT = 5
LEFT_BONE_INDEXES = [TsBone2dIndex.LeftThumbProximal, TsBone2dIndex.LeftIndexProximal,
                     TsBone2dIndex.LeftMiddleProximal, TsBone2dIndex.LeftRingProximal,
                     TsBone2dIndex.LeftLittleProximal]
RIGHT_BONE_INDEXES = [TsBone2dIndex.RightThumbProximal, TsBone2dIndex.RightIndexProximal,
                      TsBone2dIndex.RightMiddleProximal, TsBone2dIndex.RightRingProximal,
                      TsBone2dIndex.RightLittleProximal]

class TsMagneticEncoder:
    def __init__(self, lib, device, side):
        self.__device = device
        self.side = side
        self.__lib = lib
        self.__is_data_ready = False
        self.__is_started = False
        self.__positions = TsGloveMEPosition(self.side)
        self.__data_callback = None
        self.data_callback = None

    def __str__(self):
        return str(self.__positions)

    def __subscribe_on_data_update(self):
        def on_update_callback(device_ptr, data_ptr, user_data_ptr):
            for bone, position in self.__positions.fingers.items():
                flexion_angle = c_float()
                abduction_angle = c_float()
                self.__lib.ts_force_feedback_get_flexion_angle(c_void_p(data_ptr), bone.value, pointer(flexion_angle))                
                self.__lib.ts_force_feedback_get_abduction_angle(c_void_p(data_ptr), bone.value, pointer(abduction_angle))
                position.flexion_angle = flexion_angle.value
                position.abduction_angle = abduction_angle.value
            if self.data_callback != None:
                self.data_callback(self.__positions)

        me_update_prototype = CFUNCTYPE(None, c_void_p, c_void_p, c_void_p)
        self.__data_callback = me_update_prototype(on_update_callback)
        self.__lib.ts_force_feedback_set_position_update_callback(self.__device, self.__data_callback, c_void_p())

    def set_data_update_callback(self, callback):
        self.data_callback = callback

    def get_positions(self):
        return self.__positions

    def start_me_streaming(self):
        if self.__is_started:
            return
        self.__subscribe_on_data_update()
        self.__lib.ts_force_feedback_start_position_streaming(self.__device)
        self.__is_started = True

    def stop_me_streaming(self):
        if not self.__is_started:
            return
        self.__lib.ts_force_feedback_stop_position_streaming(self.__device)
        self.__is_started = False

    def ts_force_feedback_enable(self, ff_configs):
        control_casted = (TsForceFeedbackConfig * len(ff_configs))(*ff_configs)
        self.__lib.ts_force_feedback_enable(self.__device,
                                            cast(control_casted, POINTER(TsForceFeedbackConfig)),
                                            c_uint64(len(ff_configs)))

    def ts_force_feedback_disable(self, bone_indexes):
        bone_indexes_casted = (c_uint32 * len(bone_indexes))(*bone_indexes)
        self.__lib.ts_force_feedback_disable(self.__device,
                                             pointer(bone_indexes_casted),
                                             c_uint64(len(bone_indexes)))

    def get_default_position_struct(self):
        if self.side == ts_types.TsDeviceSide.Left:
            return get_left_default_position_struct()
        else:
            return get_right_default_position_struct()

    def get_default_ff_controls_struct(self):
        if self.side == ts_types.TsDeviceSide.Left:
            return get_left_default_ff_controls_struct()
        else:
            return get_right_default_ff_controls_struct()

    def get_bone_indexes(self):
        if self.side == ts_types.TsDeviceSide.Left:
            return LEFT_BONE_INDEXES
        else:
            return RIGHT_BONE_INDEXES

# utils

def get_position_struct_for_bones(bone_indexes):
    positions = dict()
    for index in bone_indexes:
        positions[index] = TsFingerMEPosition()
    return positions

def get_left_default_position_struct():
    return get_position_struct_for_bones(LEFT_BONE_INDEXES)

def get_right_default_position_struct():
    return get_position_struct_for_bones(RIGHT_BONE_INDEXES)

def mirror_bone_index(bone_index):
    for i in range(0, len(LEFT_BONE_INDEXES)):
        if LEFT_BONE_INDEXES[i] == bone_index:
            return RIGHT_BONE_INDEXES[i]
        if RIGHT_BONE_INDEXES[i] == bone_index:
            return LEFT_BONE_INDEXES[i]
    return bone_index

def get_ff_controls_struct_for_bones(bone_indexes):    
    ff_controls = (TsForceFeedbackConfig * len(bone_indexes))()
    for i in range(0, len(bone_indexes)):
        ff_controls[i].bone_index = bone_indexes[i]
    return ff_controls

def get_left_default_ff_controls_struct():
    return get_ff_controls_struct_for_bones(LEFT_BONE_INDEXES)

def get_right_default_ff_controls_struct():
    return get_ff_controls_struct_for_bones(RIGHT_BONE_INDEXES)
