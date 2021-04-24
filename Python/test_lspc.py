# Create a virtual serial port by writing
#   sudo socat PTY,link=/dev/ttyS18 PTY,link=/dev/ttyS19
#   chmod a+rw /dev/ttyS18
#   chmod a+rw /dev/ttyS19
# Now there is a virtual serial tunnel between /dev/ttyS18 and /dev/ttyS19
# Also make sure that user is in the 'dialout' group
# You can test the serial connection with 'cu'
#   cu -s 9600 -l /dev/ttyS18
# To exit press ~~.   (escape character in cu is ~)

def Callback(data, params):
    lspc = params
    lspc.transmit(0x02, data)
    print(data)

import lspc
import time

#packet_data = bytearray((0x00, 0x01, 0x05, 0x05, 0xDD, 0xFF, 0xFF, 0xFF))
#packet = lspc.Packet(packet_data)

ports = lspc.list_serial_ports()
a = lspc.LSPC(ports[1])

a.registerCallback(0x01, Callback, (a))

a.open()
time.sleep(3)
a.close()

#ports = lspc.list_serial_ports()
#b = lspc.LSPC(ports[1])

#%%
#a.ser.close()
#b.ser.close()