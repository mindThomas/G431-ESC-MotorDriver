from ctypes import *
from threading import Thread
import time


class VciInitConfig(Structure):
    _fields_ = [("AccCode", c_ulong),
                ("AccMask", c_ulong),
                ("Reserved", c_ulong),
                ("Filter", c_ubyte),
                # SJA1000
                ("Timing0", c_ubyte),  # BTR0
                ("Timing1", c_ubyte),  # BTR1
                ("Mode", c_ubyte)]


class VciCanObj(Structure):
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_uint),
                ("TimeFlag", c_ubyte),
                ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte),
                ("ExternFlag", c_ubyte),
                ("DataLen", c_ubyte),
                ("Data", c_ubyte * 8),
                ("Reserved", c_ubyte * 3)]


class CANBus:
    def __init__(self, bus_port=0):
        if (bus_port != 0 & bus_port != 1):
            print('Incorrect Bus port. Should be 0 or 1')
            return  # error

        self.dll = windll.LoadLibrary('./ControlCAN.dll')  # 调用dll文件
        self.nDeviceType = 4  # USBCAN-2E-U
        self.nDeviceInd = 0
        self.nReserved = 0
        self.nCANInd = bus_port

        # Init CAN bus interface
        self.config = VciInitConfig()
        self.config.AccCode = 0x00000000
        self.config.AccMask = 0xffffffff
        self.config.reserved = 0
        self.config.Filter = 0
        self.config.Timing0 = 0x00  # 1000Kbps  (see DLL manual)
        self.config.Timing1 = 0x14  # 1000Kbps
        self.config.Mode = 0

        self.callbacks = {}
        self.isOpen = False

    def open(self):
        # OpenDevice
        ret = self.dll.VCI_OpenDevice(self.nDeviceType, self.nDeviceInd, self.nReserved)
        print("VCI_OpenDevice =", ret)
        if (ret < 0):
            return ret

        # InitCAN
        ret = self.dll.VCI_InitCAN(self.nDeviceType, self.nDeviceInd, self.nCANInd, byref(self.config))
        print("VCI_InitCAN =", ret)
        if (ret < 0):
            return ret

        # StartCAN
        ret = self.dll.VCI_StartCAN(self.nDeviceType, self.nDeviceInd, self.nCANInd)
        print("VCI_StartCAN = ", ret)

        if (ret == 1):
            self.isOpen = True

        self.processingThread = Thread(target=self.__process__)
        self.processingThread.start()

        return ret

    def close(self):
        ret = self.dll.VCI_CloseDevice(self.nDeviceType, self.nDeviceInd)
        print("VCI_CloseDevice =", ret)

        if (ret == 1):
            self.isOpen = False

        return ret

    def registerCallback(self, ID, callback, param):
        if isinstance(ID, int):
            id = ID
        elif isinstance(ID, str):
            id = int(ID, 16)

        self.callbacks[id] = (callback, param)

    def unregisterCallback(self, ID):
        if isinstance(ID, int):
            id = ID
        elif isinstance(ID, str):
            id = int(ID, 16)

        self.callbacks.pop(id)

    def __process__(self):
        rxPackage = VciCanObj()
        while self.isOpen:
            ret = 1
            while ret:  # receive data until there is no more
                ret = self.dll.VCI_Receive(self.nDeviceType, self.nDeviceInd, self.nCANInd, byref(rxPackage), 1, 0)
                #ID = hex(rxPackage.ID)
                #print('[%s]' % (ID))
                if rxPackage.ID in self.callbacks:
                    callback, param = self.callbacks.get(rxPackage.ID)
                    data = list(rxPackage.Data)
                    callback(data[0:rxPackage.DataLen], param)
            time.sleep(0.050)

    def flush(self):
        ret = self.dll.VCI_ClearBuffer(self.nDeviceType, self.nDeviceInd, self.nCANInd)
        print("VCI_ClearBuffer =", ret)
        return ret

    def available(self):
        ret = self.dll.VCI_GetReceiveNum(self.nDeviceType, self.nDeviceInd, self.nCANInd)
        print("VCI_GetReceiveNum =", ret)
        return ret

    def transmit(self, ID, data):
        if isinstance(ID, int):
            id = ID
        elif isinstance(ID, str):
            id = int(ID, 16)

        txPackage = VciCanObj()
        txPackage.ID = id
        txPackage.SendType = 0
        txPackage.RemoteFlag = 0
        txPackage.ExternFlag = 0
        txPackage.DataLen = len(data)
        if len(data) < 8:
            data.extend([0] * (8-len(data))) # pad data vector with 0's
        txPackage.Data = tuple(data)
        txPackage.Reserved = (0, 0, 0)

        ret = self.dll.VCI_Transmit(self.nDeviceType, self.nDeviceInd, self.nCANInd, byref(txPackage), 1)
        return ret