from ctypes import (c_void_p, c_uint32,
                    c_uint64, c_float,
                    c_bool, POINTER,
                    pointer, Structure, cast)
from enum import Enum, unique


@unique
class TsHapticParamType(Enum):
    Undefined = 0
    Period = 1
    Amplitude = 2
    PulseWidth = 3
    Temperature = 4


class TsHapticParam(Structure):
    _pack_ = 1
    _fields_ = [('type', c_uint32),
                ('value', c_uint64)]


class TsHapticParamMultiplier(Structure):
    _pack_ = 1
    _fields_ = [('type', c_uint32),
                ('value', c_float)]


class TsHapticPlayer:
    """
    A player for haptic assets and instant touches. Player is binded to single device

    """

    def __init__(self, __lib, device):
        self.__device = device
        self.__lib = __lib

    def is_player_running(self):
        is_running = c_bool(False)
        self.__lib.ts_haptic_is_player_running(self.__device, pointer(is_running))
        return is_running.value

    def stop_player(self):
        self.__lib.ts_haptic_stop_player(self.device_handle)

    def is_player_paused(self):
        is_player_paused = c_bool(False)
        self.__lib.ts_haptic_get_player_paused(self.__device, pointer(is_player_paused))

        return is_player_paused.value

    def set_player_paused(self, is_paused):
        self.__lib.ts_haptic_set_player_paused(self.__device, c_bool(is_paused))

    def is_player_muted(self):
        is_player_muted = c_bool(False)

        self.__lib.ts_haptic_get_player_muted(self.__device, pointer(is_player_muted))
        return is_player_muted.value

    def set_player_muted(self, is_muted):
        self.__lib.ts_haptic_set_player_muted(self.__device, c_bool(is_muted))

    def get_player_time(self):
        player_time = c_uint64(0)

        self.__lib.ts_haptic_get_player_muted(self.__device, pointer(player_time))
        return player_time.value

    def get_number_of_master_multipliers(self):
        number = c_uint64(0)
        self.__lib.ts_haptic_get_number_of_master_multipliers(self.__device, pointer(number))

        self.number_of_master_multipliers = number.value
        return number.value

    def get_master_multipliers(self):
        number = self.get_number_of_master_multipliers()

        self.master_multipliers = (TsHapticParamMultiplier * number)()
        self.__lib.ts_haptic_get_master_multipliers(self.__device, pointer(self.master_multipliers),
                                                    c_uint64(number))
        return [*self.master_multipliers]

    def set_master_multipliers(self, multipliers):
        number = self.get_number_of_master_multipliers()
        if len(multipliers) != number:
            print(
                f'TsHapticPlayer set_master_multiplier not valid argument: {multipliers}; expecting {number} multipliers')
            return

        data = cast((TsHapticParamMultiplier * number)(), POINTER(TsHapticParamMultiplier))
        for i, param_multilpier in enumerate(multipliers):
            data[i].type = param_multilpier.type
            data[i].value = param_multilpier.value

        self.__lib.ts_haptic_set_master_multipliers(self.__device, data, c_uint64(number))

    def get_master_multiplier(self, type):
        multipliers = self.get_master_multipliers()
        for m in multipliers:
            if m.type == type.value:
                return m

        print(f'TsHapticPlayer get_master_multiplier failed to find multiplier of type: {type}')
        return TsHapticParamMultiplier(0, 0)

    def set_master_multiplier(self, multiplier):
        multipliers = self.get_master_multipliers()
        for m in multipliers:
            if m.type == multiplier.type:
                m.value = multiplier.value
                self.set_master_multipliers(multipliers)
                return
        print(f'TsHapticPlayer set_master_multiplier failed to find multiplier of type: {type}')

    def create_playable(self, asset, is_looped):
        playable_id = c_uint64(0)
        self.__lib.ts_haptic_create_playable_from_asset(self.__device, c_void_p(asset),
                                                        c_bool(is_looped), pointer(playable_id))
        return playable_id.value

    def is_playable_exists(self, playable_id):
        is_exists = c_bool(False)

        self.__lib.ts_haptic_is_playable_exists(self.__device, c_uint64(playable_id), pointer(is_exists))
        return is_exists.value

    def play_playable(self, playable_id):
        self.__lib.ts_haptic_play_playable(self.__device, c_uint64(playable_id))

    def play_touch(self, params, channels, duration):
        params_casted = cast((TsHapticParam * len(params))(), POINTER(TsHapticParam))
        for i, param in enumerate(params):
            params_casted[i].type = param.type
            params_casted[i].value = param.value

        channels_casted = cast((c_void_p * len(channels))(), POINTER(c_void_p))
        for i, channel in enumerate(channels):
            channels_casted[i] = channel

        self.__lib.ts_haptic_play_touch(self.__device, params_casted,
                                        c_uint64(len(params)), channels_casted,
                                        c_uint64(len(channels)), c_uint64(duration))

    def create_touch(self, params, channels, duration):
        playable_id = c_uint64(0)
        self.__lib.ts_haptic_create_touch(self.__device, (TsHapticParam * len(params))(*params),
                                          len(params), (c_void_p * len(channels))(*channels),
                                          len(channels), c_uint64(duration), pointer(playable_id))
        return playable_id.value

    def is_playable_playing(self, playable_id):
        is_playing = c_bool(False)

        self.__lib.ts_haptic_is_playable_playing(self.__device, c_uint64(playable_id), pointer(is_playing))
        return bool(is_playing)

    def stop_playable(self, playable_id):
        self.__lib.ts_haptic_stop_playable(self.__device, c_uint64(playable_id))

    def remove_playable(self, playable_id):
        self.__lib.ts_haptic_remove_playable(self.__device, c_uint64(playable_id))

    def get_playable_paused(self, playable_id):
        is_paused = c_bool(False)

        self.__lib.ts_haptic_get_playable_paused(self.__device, c_uint64(playable_id), pointer(is_paused))
        return bool(is_paused)

    def set_playable_paused(self, playable_id, is_paused):
        self.__lib.ts_haptic_set_playable_paused(self.__device, c_uint64(playable_id), c_bool(is_paused))

    def get_playable_muted(self, playable_id):
        is_muted = c_bool(False)

        self.__lib.ts_haptic_get_playable_muted(self.__device, c_uint64(playable_id), pointer(is_muted))
        return bool(is_muted)

    def set_playable_muted(self, playable_id, is_muted):
        self.__lib.ts_haptic_set_playable_muted(self.__device, c_uint64(playable_id), c_bool(is_muted))

    def get_playable_looped(self, playable_id):
        is_looped = c_bool(False)

        self.__lib.ts_haptic_get_playable_looped(self.__device, c_uint64(playable_id), pointer(is_looped))
        return bool(is_looped)

    def set_playable_looped(self, playable_id, is_looped):
        self.__lib.ts_haptic_set_playable_looped(self.__device, c_uint64(playable_id), c_bool(is_looped))

    def get_number_of_playable_multipliers(self, playable_id):
        number = c_uint64(0)
        self.__lib.ts_haptic_get_number_of_playable_multipliers(self.__device, c_uint64(playable_id),
                                                                pointer(number))
        return number.value

    def get_playable_multipliers(self, playable_id):
        number = self.get_number_of_playable_multipliers(playable_id)
        multipliers = (TsHapticParamMultiplier * number.value)()
        self.__lib.ts_haptic_get_playable_multipliers(self.__device, c_uint64(playable_id),
                                                      pointer(multipliers), c_uint64(number.value))
        return multipliers

    def set_playable_multipliers(self, playable_id, multipliers):
        number = len(multipliers)
        playable_multipliers = (TsHapticParamMultiplier * number)(*multipliers)
        self.__lib.ts_haptic_set_playable_multipliers(self.__device, c_uint64(playable_id),
                                                      pointer(playable_multipliers), c_uint64(number))

    def get_playable_local_time(self, playable_id):
        local_time = c_uint64(0)
        self.__lib.ts_haptic_get_playable_local_time(self.__device, c_uint64(playable_id),
                                                     pointer(local_time))
        return int(local_time.value)

    def set_playable_local_time(self, playable_id, local_time):
        self.__lib.ts_haptic_set_playable_local_time(self.__device, c_uint64(playable_id),
                                                     c_uint64(local_time))

    def get_playable_duration(self, playable_id):
        duration = c_uint64(0)
        self.__lib.ts_haptic_get_playable_duration(self.__device, c_uint64(playable_id), pointer(duration))
        return int(duration.value)

    def clear_all_playables(self):
        self.__lib.ts_haptic_clear_all_playables(c_void_p(self.__device))

    def add_channel_to_dynamic_playable(self, channel_id, playable_id):
        self.__lib.ts_haptic_add_channel_to_dynamic_playable(self.__device, pointer(channel_id),
                                                             c_uint64(playable_id))

    def remove_channel_from_dynamic_playable(self, channel_id, playable_id):
        self.__lib.ts_haptic_remove_channel_from_dynamic_playable(self.__device, pointer(channel_id),
                                                                  c_uint64(playable_id))

    def set_material_channel_impact(self, channel_id, impact, playable_id):
        self.__lib.ts_haptic_set_material_channel_impact(self.__device, pointer(channel_id),
                                                         c_float(impact), c_uint64(playable_id))

    # haptic utils

    def create_touch_parameters(self, period_ms, amplitude, pulse_width):
        return [TsHapticParam(TsHapticParamType.Period.value, period_ms),
                TsHapticParam(TsHapticParamType.Amplitude.value, amplitude),
                TsHapticParam(TsHapticParamType.PulseWidth.value, pulse_width)]

    def create_touch_multipliers(self, period_ms_m, amplitude_m, pulse_width_m):
        return [TsHapticParamMultiplier(TsHapticParamType.Period.value, period_ms_m),
                TsHapticParamMultiplier(TsHapticParamType.Amplitude.value, amplitude_m),
                TsHapticParamMultiplier(TsHapticParamType.PulseWidth.value, pulse_width_m)]
