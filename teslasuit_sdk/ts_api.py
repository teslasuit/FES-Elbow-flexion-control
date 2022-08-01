#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from teslasuit_sdk import ts_loader
from teslasuit_sdk import ts_device_manager
from teslasuit_sdk.ts_mapper import TsMapper
from teslasuit_sdk.ts_asset_manager import TsAssetManager

class TsApi:
    """
    An aggregate TESLASUIT API class that contains C API library loader and device manager.

    Attributes
    ----------
    lib : CDLL
        a loaded TESLASUIT C API library object
    version : TsVersion
        the version of loaded and used C API
    device_manager : TsDeviceManager
        a class that provides access to TESLASUIT devices
    """
    def __init__(self, lib_path=None):
        self.__lib = None
        self.__initialized = False

        self.__load_library(lib_path)
        self.ts_initialize()

        self.mapper = TsMapper(self.__lib)
        self.asset_manager = TsAssetManager(self.__lib)
        self.__device_manager = ts_device_manager.TsDeviceManager(self.__lib)

    def __load_library(self, lib_path=None):
        loader = ts_loader.TsLoader(lib_path)
        self.__lib = loader.load()

    def __unload_library(self):
        del self.__lib

    def ts_initialize(self):
        if self.__initialized:
            return

        self.__lib.ts_initialize()
        self.__initialized = True
        print('TS API initialized')

    def ts_uninitialize(self):
        if not self.__initialized:
            return

        self.__lib.ts_uninitialize()
        self.__initialized = False
        print('TS API uninitialized')

    def get_device_manager(self):
        return self.__device_manager

    def __del__(self):
        self.__device_manager.stop()
        self.ts_uninitialize()
        self.__unload_library()
