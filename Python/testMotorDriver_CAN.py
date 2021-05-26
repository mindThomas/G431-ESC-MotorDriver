import time

from CANBus import CANBus

can = CANBus()
can.open()

i = 2
while i:
    can.transmit("0x101", [0, 50])
    time.sleep(0.5)
    can.transmit("0x101", [1, 50])
    time.sleep(0.5)
    i -= 1

can.transmit("0x101", [0, 0])

can.close()
