import sys
import os
ts_api_path = os.environ['TESLASUIT_PYTHON_API_PATH']
sys.path.append(ts_api_path)
from teslasuit_sdk import ts_api


class Teslasuit():
    """
    Connector class that creates object with simple inteface for working with Teslasuit API.

        Attributes
        ----------
        api : TsApi ('Left', 'Right')
            Teslasuit API obj.
        haptic : TsHapticPlayer
            Teslasuit haptic player obj.
        streamer : str
            Teslasuit MoCap streamer obj.
        ems_channels : dict
            Dictionary containing Teslasit EMS mapping

        Methods
        -------
        connect_suit():
            Connects to the Teslasuit and initalize subsystems
        haptic_play_touch(channel_id, ampl = 100, period = 100, pw = 100, duration = 100)
            Send an electrical stimulation on the appropriate channel
        stop_player():
            Stops all current playing touches (electrical stimulation patterns)
        start_mocap_streaming():
            Starts mocap streaming
        stop_mocap_streaming():
            Stops mocap streaming
        get_raw_mocap():
            Returns raw MoCap buffer data from all IMUs
        get_q6(index):
            Returns current quternion values from the appropriate TsBoneIndex
        start_emg_streaming(buffer):
            Starts EMG streaming
        stop_emg_streaming():
            Stops EMG streaming
        get_current_emg_channel_data(node,channel)
            Returns current EMG buffer data on the appropriate channel of the selected node
    """

    def __init__(self):
        self.api = ts_api.TsApi()

    def connect_suit(self):
        device_manager = self.api.get_device_manager()
        suit = device_manager.get_or_wait_last_device_attached()
        print('Suit is connected.')

        self.streamer = suit.mocap
        self.haptic = suit.haptic
        mapper = self.api.mapper

        mapping_handle = suit.get_mapping()
        layouts = mapper.get_layouts(mapping_handle)
        for i in range(0, len(layouts)):
            if mapper.get_layout_element_type(ayouts[i]) == 2 and mapper.get_layout_type(layouts[i]) == 1:
                break
        bones = mapper.get_layout_bones(layouts[i])

        self.ems_channels = {}
        self.ems_channels['Left'] = {}
        self.ems_channels['Right'] = {}
        self.ems_channels['Right']['Biceps'] = [mapper.get_bone_contents(bones[14])[1]]
        self.ems_channels['Right']['Triceps'] = [mapper.get_bone_contents(bones[15])[1]]
        self.ems_channels['Left']['Biceps'] = [mapper.get_bone_contents(bones[12])[1]]
        self.ems_channels['Left']['Triceps'] = [mapper.get_bone_contents(bones[13])[1]]

    def haptic_play_touch(
            self,
            channel_id,
            ampl=100,
            period=20000,
            pw=100,
            duration=100):
        """
        Parameters
        ----------
        channel_id : c_int or list
            Index(es) of haptic channels to send stimulation on
        ampl : int
            Stimulation amplitude in mA
        period: int
            Stimulation period in mSec.
        pw:
            Pulse width in mkSec.
        duration
            Stimulation duration in mSec.
        """

        params = self.haptic .create_touch_parameters(period, ampl, pw)
        playable_id = self.haptic.create_touch(params, channel_id, duration)
        self.haptic.play_playable(playable_id)

    def stop_player(self):
        print('Stop player')
        self.haptic.stop_player()

    def start_mocap_streaming(self):
        self.streamer.start_streaming()

    def stop_mocap_streaming(self):
        self.streamer.stop_mocap_streaming()

    def get_raw_mocap(self):
        return self.streamer.get_raw_data_on_ready()

    def get_q6(self, index):
        """
        Parameters
        ----------
        index : TsBoneIndex
            Index of the TsBone to get quat. from

        Returns
        -------
        list
            A list containing quat x,y,z and w values
        """

        last_imu_data = self.streamer.get_raw_data_on_ready()
        q6_data = [
            last_imu_data[index].q6.x,
            last_imu_data[index].q6.y,
            last_imu_data[index].q6.z,
            last_imu_data[index].q6.w]
        return q6_data

    def start_emg_streaming(self, buffer=100):
        """
        Parameters
        ----------
        buffer : int
            size of the EMG data buffer
        """

        self.streamer.is_emg_buffering_enabled = True
        self.streamer.emg_buffer_size = buffer
        print('Start emg...')
        self.streamer.start_emg_streaming()

    def stop_emg_streaming(self):
        print('Stop emg')
        self.streamer.stop_mocap_streaming()

    def get_current_emg_channel_data(self, node, channel):
        """
        Parameters
        ----------
        node: int
            Node index
        channel: int
            Channel index

        Returns
        -------
        list
            A list containing EMG data buffer
            for the appropriate channel of selected node.
        """

        return self.streamer.emg_buffer.nodes[node].channels[channel].buffer
