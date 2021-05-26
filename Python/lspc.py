import glob
import sys
import time
from queue import Queue
from threading import Thread

import serial  # https://pyserial.readthedocs.io/en/latest/shortintro.html
from cobs import cobs

# Valuable Python guide: https://docs.python-guide.org/writing/structure/

# Global defines
STATE_HEADER = 0
STATE_TYPE = 1
STATE_LENGTH = 2
STATE_DATA = 3

# General functions
"""" List available serial ports """


def list_serial_ports():
    """Lists serial port names

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of the serial ports available on the system
    """
    if sys.platform.startswith("win"):
        ports = ["COM%s" % (i + 1) for i in range(256)]
    elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob("/dev/tty[A-Za-z]*")
    elif sys.platform.startswith("darwin"):
        ports = glob.glob("/dev/tty.*")
    else:
        raise EnvironmentError("Unsupported platform")

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException) as err:
            # print("OS error: ", err)
            # print("OS error: {0}".format(err))
            pass
    return result


# Packet encoder and decoder
class Packet:
    @staticmethod
    def encode(id, payload):
        package = bytearray(len(payload) + 4)

        # Package header
        package[0] = 0

        # Package ID/Type
        if id == 0:  # error
            return bytearray()
        package[1] = id

        # Package length
        if len(payload) > 254:  # error
            return bytearray()
        package[2] = len(payload) + 1

        # Encode payload
        encoded_data = cobs.encode(payload)
        package[3:] = encoded_data

        return package

    @staticmethod
    def decode(package):
        # Check package length
        if len(package) < 4:  # error
            return 0, bytearray()

        # Check package length
        if len(package) > 258:  # error
            return 0, bytearray()

        # Extract and check package ID/Type
        id = package[1]
        if id == 0:  # error
            return 0, bytearray()

        # Verify package length
        if len(package) != package[2] + 3:  # error
            return 0, bytearray()

        # Decode payload
        payload_length = len(package) - 4  # COB = Consistent Overhead Byte Stuffing
        try:
            data = cobs.decode(package[3:])

            if len(data) != payload_length:
                return 0, bytearray()

            return id, data
        except:
            return 0, bytearray()


# LSPC object
class LSPC:
    def __init__(self, port):
        self.callbacks = {}
        self.port = port
        self.isOpen = False

        self.rx_state = STATE_HEADER
        self.flush_now = False

    def open(self, baudrate=115200):
        try:
            self.ser = serial.Serial(self.port)
            self.ser.baudrate = baudrate
            self.ser.parity = serial.PARITY_NONE
            self.ser.bytesize = 8
            self.ser.stopbits = 1
            self.ser.timeout = 0.1  # wait 100 ms for messages

            self.isOpen = self.ser.is_open
            self.flush_now = False

            self.rx_state = STATE_HEADER
            self.tx_queue = Queue()

            self.rx_processing_thread = Thread(target=self.__rx_process__)
            self.rx_processing_thread.start()

            self.tx_processing_thread = Thread(target=self.__tx_process__)
            self.tx_processing_thread.start()

        except:
            print("Error opening serial port")

    def close(self):
        try:
            self.ser.close()
            self.isOpen = self.ser.is_open
        except:
            self.isOpen = self.ser.is_open

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

    def __rx_process__(self):
        while self.isOpen:
            # bytes_read = self.ser.read_until(terminator=[0x00])
            bytes_read = self.ser.read_all()
            if len(bytes_read) > 0:
                # print('Read %d bytes' % (len(bytes_read)))
                for b in bytes_read:
                    self.processIncomingByte(b)
                    if self.flush_now:
                        self.flush_now = False
                        break

            time.sleep(0.005)

    def flush(self):
        self.ser.flush()
        self.ser.read_all()
        self.flush_now = True
        while self.flush_now:
            time.sleep(0.050)

    def processIncomingByte(self, b):
        if self.rx_state == STATE_HEADER:
            if b == 0x00:
                self.rx_data = bytearray()
                self.rx_data.append(b)
                self.rx_state = STATE_TYPE

        elif self.rx_state == STATE_TYPE:
            if b != 0x00:
                self.rx_data.append(b)
                self.rx_state = STATE_LENGTH
            else:  # incorrect byte at this point = reset state
                self.rx_state = STATE_HEADER

        elif self.rx_state == STATE_LENGTH:
            self.rx_length = b
            self.rx_data.append(b)
            self.rx_state = STATE_DATA

        elif self.rx_state == STATE_DATA:
            self.rx_data.append(b)

            # If we got it all, decode it and invoke the handler
            if len(self.rx_data) >= self.rx_length + 3:
                id, data = Packet.decode(self.rx_data)

                if id in self.callbacks:
                    callback, param = self.callbacks.get(id)
                    callback(data, param)

                self.rx_data = bytearray()
                self.rx_state = STATE_HEADER

    def __tx_process__(self):
        while self.isOpen:
            while not self.tx_queue.empty():
                package = self.tx_queue.get()
                print("Transmitting %d bytes" % (len(package)))
                self.ser.write(package)
            time.sleep(0.050)

    def transmit(self, id, data):
        package = Packet.encode(id, data)
        self.tx_queue.put(package)


if __name__ == "__main__":
    print(list_serial_ports())
