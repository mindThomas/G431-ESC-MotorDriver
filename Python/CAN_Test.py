import time
from ctypes import *

dll = windll.LoadLibrary("./ControlCAN.dll")  # 调用dll文件
nDeviceType = 4  # 设备类型USBCAN-2E-U
nDeviceInd = 0  # 索引号0，代表设备个数
nReserved = 0  # 无意义参数
# nCANInd = 1  # can通道号

# 定义一个python的'结构体'，使用ctypes继承Structure，内容是初始化需要的参数，依据产品手册
class VciInitConfig(Structure):
    _fields_ = [
        ("AccCode", c_ulong),  # 验收码，后面是数据类型
        ("AccMask", c_ulong),  # 屏蔽码
        ("Reserved", c_ulong),  # 保留
        ("Filter", c_ubyte),  # 滤波使能。0=不使能，1=使能使能时，/
        # 请参照SJA1000验收滤波器设置验收码和屏蔽码。
        ("Timing0", c_ubyte),  # 波特率定时器0（BTR0）
        ("Timing1", c_ubyte),  # 波特率定时器1（BTR1)
        ("Mode", c_ubyte),
    ]  # 模式。=0为正常模式，=1为只听模式， =2为自发自收模式


# 定义发送报文的结构体
class VciCanObj(Structure):
    _fields_ = [
        ("ID", c_uint),  # 报文帧ID'''
        ("TimeStamp", c_uint),  # 接收到信息帧时的时间标识
        ("TimeFlag", c_ubyte),  # 是否使用时间标识， 为1时TimeStamp有效
        ("SendType", c_ubyte),  # 发送帧类型。=0时为正常发送,=1时为单次发送（不自动重发)，/
        # =2时为自发自收（用于测试CAN卡是否损坏） ， =3时为单次自发自收（只发送一次， 用于自测试），/
        # 只在此帧为发送帧时有意义。
        ("RemoteFlag", c_ubyte),  # 是否是远程帧。=0时为数据帧，=1时为远程帧。
        ("ExternFlag", c_ubyte),  # 是否是扩展帧。=0时为标准帧（11位帧ID），=1时为扩展帧（29位帧ID）。
        ("DataLen", c_ubyte),  # 数据长度DLC(<=8)， 即Data的长度
        ("Data", c_ubyte * 8),  # CAN报文的数据。 空间受DataLen的约束。
        ("Reserved", c_ubyte * 3),
    ]  # 系统保留


# 定义一个用于初始化的实例对象vic
vic = VciInitConfig()
vic.AccCode = 0x00000000
vic.AccMask = 0xFFFFFFFF
vic.reserved = 0
vic.Filter = 0
vic.Timing0 = 0x00  # 1000Kbps  (see DLL manual)
vic.Timing1 = 0x14  # 1000Kbps
vic.Mode = 0

# 定义报文实例对象，用于发送
vco = VciCanObj()
vco.ID = 0x00000100  # 帧的ID
vco.SendType = 1  # 发送帧类型，0是正常发送，1为单次发送，这里要选1！要不发不去！
vco.RemoteFlag = 0
vco.ExternFlag = 0
vco.DataLen = 1
vco.Data = (0x00, 0, 0, 0, 0, 0, 0, 0)
vco.Reserved = (0, 0, 0)
# 也可以用下面的定义方式，其中报文可以用0x的表达选择是16进制还是10进制
"""
ubyte_array = c_ubyte * 8
a = ubyte_array(1, 2, 3, 4, 5, 6, 7, 0x64)
ubyte_3array = c_ubyte * 3
b = ubyte_3array(0, 0, 0)
vco = VciCanObj(0x01, 0, 0, 1, 0, 0, 8, a)
"""
# 定义报文实例对象，用于接收
vco2 = VciCanObj()
vco2.ID = 0x00000101  # 帧的ID 后面会变成真实发送的ID
vco2.SendType = 0  # 这里0就可以收到
vco2.RemoteFlag = 0
vco2.ExternFlag = 0
vco2.DataLen = 8
vco2.Data = (0, 0, 0, 0, 0, 0, 0, 0)

"""设备的打开如果是双通道的设备的话，可以再用initcan函数初始化"""
# OpenDevice(设备类型号，设备索引号，参数无意义)
ret = dll.VCI_OpenDevice(nDeviceType, nDeviceInd, nReserved)
print("opendevice:", ret)

# InitCAN(设备类型号，设备索引号，第几路CAN，初始化参数initConfig)，
ret = dll.VCI_InitCAN(nDeviceType, nDeviceInd, 0, byref(vic))
print("initcan0:", ret)

# StartCAN(设备类型号，设备索引号，第几路CAN)
ret = dll.VCI_StartCAN(nDeviceType, nDeviceInd, 0)
print("startcan0:", ret)

#%%
# dll.VCI_Transmit(nDeviceType, nDeviceInd, 0, byref(vco), 1)
#
# i = 100
# while i:
#     #art = dll.VCI_Transmit(nDeviceType, nDeviceInd, 0, byref(vco), 1)  # 发送vco
#     ret = dll.VCI_Receive(nDeviceType, nDeviceInd, 0, byref(vco2), 1, 0)  # 以vco2的形式接收报文
#     if ret > 0:
#         ID = hex(vco2.ID)
#         timestamp = int.from_bytes(list(vco2.Data[0:4]), byteorder='little', signed=False) / 10000
#         volt = int.from_bytes(list(vco2.Data[5:6]), byteorder='little', signed=False) / 1000
#         print('[{}] {} s: {} V'.format(ID, timestamp, volt))
#         i -= 1
#
# dll.VCI_Transmit(nDeviceType, nDeviceInd, 0, byref(vco), 1)

while True:
    ret = dll.VCI_Receive(nDeviceType, nDeviceInd, 0, byref(vco2), 1, 0)  # 以vco2的形式接收报文
    if ret > 0:
        ID = hex(vco2.ID)
        if vco2.ID == int("0x101", 16):
            # Current Sense
            timestamp = (
                int.from_bytes(list(vco2.Data[0:4]), byteorder="little", signed=False)
                / 10000
            )
            volt = (
                int.from_bytes(list(vco2.Data[4:6]), byteorder="little", signed=False)
                / 1000
            )
        elif vco2.ID == int("0x102", 16):
            # VNH Current Sense
            timestamp = (
                int.from_bytes(list(vco2.Data[0:4]), byteorder="little", signed=False)
                / 10000
            )
            volt = (
                int.from_bytes(list(vco2.Data[4:6]), byteorder="little", signed=False)
                / 1000
            )
        elif vco2.ID == int("0x103", 16):
            # VBAT
            timestamp = (
                int.from_bytes(list(vco2.Data[0:4]), byteorder="little", signed=False)
                / 10000
            )
            volt = (
                int.from_bytes(list(vco2.Data[4:6]), byteorder="little", signed=False)
                / 1000
            )
            vbat = volt / 0.108
            print("[%s] %0.3f s: %0.3f V" % (ID, timestamp, vbat))
        elif vco2.ID == int("0x104", 16):
            # PWM output
            timestamp = (
                int.from_bytes(list(vco2.Data[0:4]), byteorder="little", signed=False)
                / 10000
            )
            volt = (
                int.from_bytes(list(vco2.Data[4:6]), byteorder="little", signed=False)
                / 1000
            )


#%%
ret = dll.VCI_CloseDevice(nDeviceType, nDeviceInd)
print("closedevice:", ret)
