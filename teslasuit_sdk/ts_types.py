import uuid
from ctypes import *
from enum import Enum, unique, IntEnum

class TsDeviceHandle(Structure):
    pass


class TsDevice(Structure):
    _pack_ = 1
    _fields_ = [('uuid', c_uint8 * 16)]

    def __str__(self):
        return str(uuid.UUID(hex=''.join(format(b, '02x') for b in self.uuid)))


class TsVersion(Structure):
    _pack_ = 1
    _fields_ = [('major', c_uint32),
                ('minor', c_uint32),
                ('patch', c_uint32),
                ('build', c_uint32)]

    def __str__(self):
        return f'{self.major}.{self.minor}.{self.patch}'

    def __repr__(self):
        return f'TsVersion({self.major}.{self.minor}.{self.patch}.{self.build})'

@unique
class TsDeviceType(IntEnum):
    Undefined = 0
    Suit = 1
    Glove = 2

@unique
class TsDeviceSide(IntEnum):
    Undefined = 0
    Left = 1
    Right = 2

