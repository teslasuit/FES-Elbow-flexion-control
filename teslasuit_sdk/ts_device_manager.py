import time
import threading
from ctypes import (pointer, c_uint32)

from teslasuit_sdk import ts_types
from teslasuit_sdk.ts_device import TsDevice

MAX_DEVICE_COUNT = 8

class TsDeviceManager:
    def __init__(self, lib):
        self.__lib = lib
        self.devices = []
        self.is_running = True
        self.thread = threading.Thread(None, self.update_device_list)
        self.thread.start()

    def stop(self):
        self.is_running = False
        self.thread.join()

    def find_device(self, ts_device):
        for device in self.devices:
            if str(device.device_uuid) == str(ts_device):
                return device
        return None

    def get_device_uuids(self):
        uuids = set()
        for device in self.devices:
            uuids.add(str(device.device_uuid))
        return uuids

    def __add_device(self, ts_device):
        self.devices.append(TsDevice(self.__lib, ts_device))
        print("Device connected:", ts_device)

    def __remove_device(self, ts_device):
        i = 0
        while i < len(self.devices):
            if str(self.devices[i].device_uuid) == str(ts_device):
                del self.devices[i]
                print("Device disconnected:", ts_device)
            else:
                i = i+1     

    def update_device_list(self):
        while self.is_running:
            refreshing_devices = (ts_types.TsDevice * MAX_DEVICE_COUNT)()
            number_of_devices_connected = c_uint32(MAX_DEVICE_COUNT)
            self.__lib.ts_get_device_list(pointer(refreshing_devices), pointer(number_of_devices_connected))
            disconnected_devices = self.get_device_uuids()
            for i in range(0, number_of_devices_connected.value):
                refreshing_device = refreshing_devices[i]
                if self.find_device(refreshing_device) == None:
                    self.__add_device(refreshing_device)
                else:
                    disconnected_devices.remove(str(refreshing_device))
            for uuid in disconnected_devices:
                self.__remove_device(refreshing_device)
            time.sleep(0.2)

    def wait_for_device_to_connect(self, number_of_devices_to_wait=1):
        while len(self.devices) < number_of_devices_to_wait:
            print('Waiting for a device connect...')
            time.sleep(1)

    def get_or_wait_last_device_attached(self):
        if len(self.devices) == 0:
            self.wait_for_device_to_connect()
        return self.devices[len(self.devices) - 1]
