import os
import sys
from ctypes import WinDLL
# from ctypes.util import find_library


class TsLoader:
    """
    A loader of TESLASUIT C API library, provides library object.

    By default load library from path from environment variable TESLASUIT_API_LIB_PATH.
    Not default library path can be specified with lib_path argument.
    """
    def __init__(self, lib_path=None):
        self.__lib = None
        assert sys.maxsize == 2**63 - 1, 'Only 64-bit systems are supported.'
        self.__path = lib_path if lib_path is not None else os.getenv('TESLASUIT_API_LIB_PATH')

    def load(self):
        if self.__path is not None and os.path.isfile(self.__path):
            self.__lib = WinDLL(self.__path)
        # else:
        #     self.__lib = WinDLL(find_library('teslasuit_api'))
        #     if self.__lib is None:
        #         print('Cannot load library. Please check your library path ( Environment variable or path argument )'
        #               ' and try again.')
        #         return None

        return self.__lib
