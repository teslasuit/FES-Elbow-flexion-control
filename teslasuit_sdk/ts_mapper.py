from ctypes import (c_void_p, c_uint8,
                    POINTER, pointer,
                    c_uint64, Structure,
                    c_float)
from enum import IntEnum, unique


class TsVec2f(Structure):
    _pack_ = 1
    _fields_ = [('x', c_float),
                ('y', c_float)]

    def __str__(self):
        return f'{self.x};{self.y}'


@unique
class TsMapping2dVersion(IntEnum):
    Undefined = 0
    Mapping_4_5_4 = 1
    Mapping_4_5_5 = 2
    Mapping_4_6_0 = 3
    MappingLeftGlove_1_0_0 = 4
    MappingRightGlove_1_0_0 = 5
    Mapping_4_5_4_Legacy = 6
    Mapping_4_5_5_Legacy = 7
    Mapping_5_0_0 = 8
    MappingLeftGlove_1_2_0 = 9
    MappingRightGlove_1_2_0 = 10
    Mapping_5_0_1 = 11
    Mapping_5_0_2 = 12
    Mapping_5_0_3 = 13
    Mapping_4_7_0 = 16


@unique
class TsLayout2dType(IntEnum):
    Undefined = 0
    Electric = 1
    Temperature = 2
    Vibration = 3
    Emg = 4
    Ecg = 5


@unique
class TsLayout2dElementType(IntEnum):
    Undefined = 0
    Cell = 1
    Channel = 2


@unique
class TsBone2dIndex(IntEnum):
    Hips = 0
    LeftUpperLeg = 1
    RightUpperLeg = 2
    LeftLowerLeg = 3
    RightLowerLeg = 4
    LeftFoot = 5
    RightFoot = 6
    Spine = 7
    Chest = 8
    UpperChest = 9
    Neck = 10
    Head = 11
    LeftShoulder = 12
    RightShoulder = 13
    LeftUpperArm = 14
    RightUpperArm = 15
    LeftLowerArm = 16
    RightLowerArm = 17
    LeftHand = 18
    RightHand = 19
    LeftThumbProximal = 20
    LeftThumbIntermediate = 21
    LeftThumbDistal = 22
    LeftIndexProximal = 23
    LeftIndexIntermediate = 24
    LeftIndexDistal = 25
    LeftMiddleProximal = 26
    LeftMiddleIntermediate = 27
    LeftMiddleDistal = 28
    LeftRingProximal = 29
    LeftRingIntermediate = 30
    LeftRingDistal = 31
    LeftLittleProximal = 32
    LeftLittleIntermediate = 33
    LeftLittleDistal = 34
    RightThumbProximal = 35
    RightThumbIntermediate = 36
    RightThumbDistal = 37
    RightIndexProximal = 38
    RightIndexIntermediate = 39
    RightIndexDistal = 40
    RightMiddleProximal = 41
    RightMiddleIntermediate = 42
    RightMiddleDistal = 43
    RightRingProximal = 44
    RightRingIntermediate = 45
    RightRingDistal = 46
    RightLittleProximal = 47
    RightLittleIntermediate = 48
    RightLittleDistal = 49


@unique
class TsBone2dSide(IntEnum):
    Undefined = 0
    Front = 1
    Back = 2


class TsBoneContentType(Structure):
    _pack_ = 1
    _fields_ = [('layout_type', c_uint8),
                ('element_type', c_uint8)]


class TsBoneId(Structure):
    _pack_ = 1
    _fields_ = [('bone_index', c_uint8),
                ('bone_side', c_uint8)]


class TsBone(Structure):
    _pack_ = 1
    _fields_ = [('id', c_uint8),
                ('bone_index', c_uint8),
                ('bone_side', c_uint8),
                ('channels', c_uint8)]


class TsLayout(Structure):
    _pack_ = 1
    _fields_ = [('id', c_uint8),
                ('layout_type', c_uint8),
                ('element_type', c_uint8),
                ('layout_index', c_uint8),
                ('bone', POINTER(TsBone))]


class TsMappingData(Structure):
    _pack_ = 1
    _fields_ = [('mapping_handle', c_void_p),
                ('layouts', POINTER(TsLayout))]


class TsMapper:
    def __init__(self, lib):
        self.__lib = lib

    def get_mapping_by_device(self, device):
        mapping = c_void_p()
        self.__lib.ts_mapping2d_get_by_device(device, pointer(mapping))

        return mapping.value

    def get_mapping_by_version(self, version):
        mapping = c_void_p(0)
        self.__lib.ts_mapping2d_get_by_version(c_uint8(version), pointer(mapping))

        return mapping

    def get_number_of_layouts(self, mapping):
        number_of_layouts = c_uint64(0)
        self.__lib.ts_mapping2d_get_number_of_layouts(c_void_p(mapping), pointer(number_of_layouts))

        return number_of_layouts.value

    def get_layouts(self, mapping):
        number_of_layouts = self.get_number_of_layouts(mapping)
        layouts = (c_void_p * number_of_layouts)()

        self.__lib.ts_mapping2d_get_layouts(c_void_p(mapping), pointer(layouts), c_uint64(number_of_layouts))
        return [*layouts]

    def get_layout_index(self, layout):
        layout_index = c_uint8(0)
        self.__lib.ts_mapping2d_layout_get_index(c_void_p(layout), pointer(layout_index))

        return layout_index.value

    def get_layout_type(self, layout):
        layout_type = c_uint8(0)
        self.__lib.ts_mapping2d_layout_get_type(c_void_p(layout), pointer(layout_type))

        return layout_type.value

    def get_layout_element_type(self, layout):
        layout_element_type = c_uint8(0)
        self.__lib.ts_mapping2d_layout_get_element_type(c_void_p(layout), pointer(layout_element_type))

        return layout_element_type.value

    def get_number_of_bones(self, layout):
        number_of_bones = c_uint64(0)
        self.__lib.ts_mapping2d_layout_get_number_of_bones(c_void_p(layout), pointer(number_of_bones))

        return number_of_bones.value

    def get_layout_bones(self, layout):
        number_of_bones = self.get_number_of_bones(layout)
        bones = (c_void_p * number_of_bones)()

        self.__lib.ts_mapping2d_layout_get_bones(c_void_p(layout), pointer(bones), c_uint64(number_of_bones))
        return [*bones]

    def get_bone_index(self, bone_handle):
        bone_index = c_uint8(0)
        self.__lib.ts_mapping2d_bone_get_index(c_void_p(bone_handle), pointer(bone_index))

        return bone_index.value

    def get_bone_side(self, bone_handle):
        bone_side = c_uint8(0)
        self.__lib.ts_mapping2d_bone_get_side(c_void_p(bone_handle), pointer(bone_side))

        return bone_side.value

    def get_bone_number_of_contents(self, bone_handle):
        number_of_contents = c_uint64(0)
        self.__lib.ts_mapping2d_bone_get_number_of_contents(c_void_p(bone_handle), pointer(number_of_contents))

        return number_of_contents.value

    def get_bone_contents(self, bone_handle):
        number_of_contents = self.get_bone_number_of_contents(bone_handle)
        bone_contents = (c_void_p * number_of_contents)()

        self.__lib.ts_mapping2d_bone_get_contents(c_void_p(bone_handle), pointer(bone_contents),
                                                  c_uint64(number_of_contents))
        return [*bone_contents]

    def get_bone_number_of_points(self, bone_handle):
        number_of_points = c_uint64(0)
        self.__lib.ts_mapping2d_bone_content_get_number_of_points(c_void_p(bone_handle), pointer(number_of_points))

        return number_of_points.value

    def get_bone_points(self, bone_handle):
        number_of_points = self.get_bone_number_of_points(bone_handle)
        points = (TsVec2f * number_of_points)()

        self.__lib.ts_mapping2d_bone_content_get_points(c_void_p(bone_handle), pointer(points),
                                                        c_uint64(number_of_points))

        return [*points]


    # mapping utils

    def get_layout_by_type(self, mapping, layout_type, layout_element_type):
        layouts = self.get_layouts(mapping)
        for layout in layouts:
            if self.get_layout_type(layout) == layout_type and self.get_layout_element_type(layout) == layout_element_type:
                return layout
        return None

    def get_haptic_electric_channel_layout(self, mapping):
        return self.get_layout_by_type(mapping, TsLayout2dType.Electric.value, TsLayout2dElementType.Channel.value)