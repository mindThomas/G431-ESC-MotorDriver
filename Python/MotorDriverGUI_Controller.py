# System, threading and others
import collections  # for deque = circular buffer

# Data storage
import csv
import math
import signal
import struct
import sys
import time
import tkinter as Tk
from datetime import datetime
from functools import partial
from threading import Thread

import matplotlib.animation as animation

# Plotting and GUI
import matplotlib.pyplot as plt
import numpy as np

# Data processing
import pandas as pd

# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg as FigureCanvasTkAgg  # for Linux?
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg as FigureCanvasTk
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk

import lspc


class Graph:
    def __init__(self, ax, bufferLength, index, name, unit):
        self.name = name
        colors = ["r", "g", "b", "m", "c", "y"]
        self.plot = ax.plot([], [], colors[index] + "-", label=name)[0]
        self.label = ax.text(
            0.80,
            0.90 - index * 0.05,
            "",
            fontdict={"color": colors[index]},
            transform=ax.transAxes,
        )
        # Prepare circular buffer for storing data for visualization
        self.buffer = collections.deque([0] * bufferLength, maxlen=bufferLength)
        self.unit = unit

    def redraw(self):
        if len(self.buffer) > 0:
            self.plot.set_data(
                -np.array(range(len(self.buffer)))[::-1], list(self.buffer)
            )
            self.label.set_text(
                "[%s] = %.2f %s" % (self.name, self.buffer[-1], self.unit)
            )


class LivePlot(Tk.Frame):
    def __init__(self, parent, bufferLength=1000):
        Tk.Frame.__init__(self, parent)

        self.bufferLength = bufferLength

        # Construct figure
        self.figure = plt.figure(figsize=(14, 5))
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlim([-self.bufferLength + 1, 0])
        self.ax.set_ylim([0, 10])
        self.ax.set_title("Motor driver measurements")
        self.ax.set_xlabel("Sample index [k]")
        self.ax.set_ylabel("Reading [V or A]")
        plt.grid(axis="y", which="major")

        # Assign figure to drawing/window canvas
        canvas = FigureCanvasTk(self.figure, self)
        canvas.draw()
        canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=True)

        self.graphs = []

        # Configure animation (refresh/redraw) of figure - refresh every 50 ms
        self.anim = animation.FuncAnimation(
            self.figure, self.__updatePlot__, interval=50
        )  # fargs has to be a tuple, e.g. fargs=(graph,graphText)

    def __updatePlot__(self, frame):
        for graph in self.graphs:
            graph.redraw()

    def addGraph(self, name, unit = '') -> collections.deque:
        graph = Graph(self.ax, self.bufferLength, len(self.graphs), name, unit)
        self.graphs.append(graph)
        return graph.buffer

    def setYlim(self, min, max):
        self.ax.set_ylim([min, max])


class Toolbar(Tk.Frame):
    def __init__(self, parent):
        Tk.Frame.__init__(self, parent)

        self.callbacks = {}

        # Duty Cycle field + button
        lbl1 = Tk.Label(self, text="Duty Cycle")
        lbl1.pack(padx=5, pady=5)
        self.DutyCycle = Tk.Entry(self, width=4)
        self.DutyCycle.insert(0, "0")  # (index, string)
        self.DutyCycle.pack(padx=5)
        SetDutyCycleButton = Tk.Button(
            self,
            text="Set Duty Cycle",
            command=lambda: self.__btnHandler__("set_dutycycle"),
        )
        SetDutyCycleButton.pack(padx=5)

        # PWM frequency, Sample frequency and Set button
        lbl2 = Tk.Label(self, text="PWM Frequency")
        lbl2.pack(padx=5, pady=5)
        self.PWMFrequency = Tk.Entry(self, width=6)
        self.PWMFrequency.insert(0, "5000")  # (index, string)
        self.PWMFrequency.pack(padx=5)
        lbl3 = Tk.Label(self, text="Sampling Frequency")
        lbl3.pack(padx=5, pady=5)
        self.SampleFrequency = Tk.Entry(self, width=6)
        self.SampleFrequency.insert(0, "100")  # (index, string)
        self.SampleFrequency.pack(padx=5)
        SetFrequenciesButton = Tk.Button(
            self,
            text="Set frequencies",
            command=lambda: self.__btnHandler__("set_frequencies"),
        )
        SetFrequenciesButton.pack(padx=5)

        lbl4 = Tk.Label(self, text="Averaging Samples")
        lbl4.pack(padx=5, pady=5)
        self.AveragingCount = Tk.Entry(self, width=6)
        self.AveragingCount.insert(0, "1")  # (index, string)
        self.AveragingCount.pack(padx=5)
        SetAveragingButton = Tk.Button(
            self,
            text="Set averaging",
            command=lambda: self.__btnHandler__("set_averaging"),
        )
        SetAveragingButton.pack(padx=5)

        lbl8 = Tk.Label(self, text="Current Setpoint")
        lbl8.pack(padx=5, pady=5)
        self.CurrentSetpoint = Tk.Entry(self, width=6)
        self.CurrentSetpoint.insert(0, "1")  # (index, string)
        self.CurrentSetpoint.pack(padx=5)
        SetCurrentSetpointButton = Tk.Button(
            self, text="Set current", command=lambda: self.__btnHandler__("set_current")
        )
        SetCurrentSetpointButton.pack(padx=5)

        lbl9 = Tk.Label(self, text="Controller")
        lbl9.pack(padx=5, pady=5)
        Horizontal3 = Tk.Frame(self)
        Horizontal3.pack(pady=5)
        self.CurrentKp = Tk.Entry(Horizontal3, width=6)
        self.CurrentKp.insert(0, "4")  # (index, string)
        self.CurrentKp.pack(padx=5, side="left")
        self.CurrentTi = Tk.Entry(Horizontal3, width=6)
        self.CurrentTi.insert(0, "10")  # (index, string)
        self.CurrentTi.pack(padx=5, side="left")
        SetCurrentKpTiButton = Tk.Button(
            Horizontal3,
            text="Set Kp/Ti",
            command=lambda: self.__btnHandler__("set_kpti"),
        )
        SetCurrentKpTiButton.pack(padx=5, side="left")

        lbl5 = Tk.Label(self, text="Sweeps")
        lbl5.pack(padx=5, pady=5)
        Horizontal2 = Tk.Frame(self)
        Horizontal2.pack(pady=5)
        StartCurrentSweepButton = Tk.Button(
            Horizontal2,
            text="Current",
            command=lambda: self.__btnHandler__("start_current_sweep"),
        )
        StartCurrentSweepButton.pack(padx=5, side="left")
        StartDutySweepButton = Tk.Button(
            Horizontal2,
            text="Duty",
            command=lambda: self.__btnHandler__("start_duty_sweep"),
        )
        StartDutySweepButton.pack(padx=5, side="left")
        StartSampleLocationSweepButton = Tk.Button(
            Horizontal2,
            text="Loc",
            command=lambda: self.__btnHandler__("start_sample_location_sweep"),
        )
        StartSampleLocationSweepButton.pack(padx=5, side="left")

        lbl6 = Tk.Label(self, text="Calibration")
        lbl6.pack(padx=5, pady=5)
        Horizontal3 = Tk.Frame(self)
        Horizontal3.pack(pady=5)
        self.CurrentCalibrationButton = Tk.Button(
            Horizontal3,
            text="Current",
            command=lambda: self.__btnHandler__("calibration_current"),
        )
        self.CurrentCalibrationButton.pack(padx=5, side="left")
        self.VbusCalibrationButton = Tk.Button(
            Horizontal3,
            text="Vbus",
            command=lambda: self.__btnHandler__("calibration_vbus"),
        )
        self.VbusCalibrationButton.pack(padx=5, side="left")
        self.BemfCalibrationButton = Tk.Button(
            Horizontal3,
            text="Bemf",
            command=lambda: self.__btnHandler__("calibration_bemf"),
        )
        self.BemfCalibrationButton.pack(padx=5, side="left")
        self.CalibrationMeasurement = Tk.Entry(self, width=6)
        self.CalibrationMeasurement.insert(0, "0")  # (index, string)
        self.CalibrationMeasurement.pack(padx=5)
        self.AddCalibrationMeasurementButton = Tk.Button(
            self,
            text="Add measurement",
            command=lambda: self.__btnHandler__("calibration_add_measurement"),
        )
        self.AddCalibrationMeasurementButton.pack(padx=5)
        self.AddCalibrationMeasurementButton["state"] = "disabled"

        # Y-limits
        lbl7 = Tk.Label(self, text="Ylim")
        lbl7.pack(padx=5, pady=5)

        Horizontal4 = Tk.Frame(self)
        Horizontal4.pack()
        self.YlimMin = Tk.Entry(Horizontal4, width=5)
        self.YlimMin.insert(0, "-0.2")  # (index, string)
        self.YlimMin.pack(padx=5, side="left")
        self.YlimMax = Tk.Entry(Horizontal4, width=5)
        self.YlimMax.insert(0, "0.8")  # (index, string)
        self.YlimMax.pack(padx=5, side="left")

        YlimButton = Tk.Button(
            self, text="Set limits", command=lambda: self.__btnHandler__("set_limits")
        )
        YlimButton.pack(padx=5)

        self.ToggleLogBtn = Tk.Button(self, text="Enable Logger", command=lambda: self.__btnHandler__("toggle_logger"))
        self.ToggleLogBtn.pack(padx=5, pady=20)

        self.PrintCPULoad = Tk.Button(self, text='CPU Load', command=lambda: self.__btnHandler__('print_cpuload'))
        self.PrintCPULoad.pack(padx=5, pady=0)

    def __btnHandler__(self, callbackName):
        # print('Button pressed')
        if callbackName in self.callbacks:
            callback, param = self.callbacks[callbackName]
            callback(param)

    def registerCallback(self, name, callback, param):
        self.callbacks[name] = (callback, param)

    def unregisterCallback(self, name):
        self.callbacks.pop(name)


class MainWindow(Tk.Frame):
    def __init__(self, parent):
        Tk.Frame.__init__(self, parent)
        # Assemble main window
        self.toolbar = Toolbar(parent)
        self.liveplot = LivePlot(parent)
        # Set locations of window widgets
        self.toolbar.pack(side=Tk.LEFT, fill=Tk.BOTH, expand=True)
        self.liveplot.pack(side=Tk.RIGHT, fill=Tk.BOTH, expand=True)


class BufferedCSVwriter:
    def __init__(self, type):
        self.filename = ""
        self.csv = 0
        self.type = type

    @staticmethod
    def getTimestampString():
        return datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "-{0:03d}".format(
            round(datetime.now().microsecond / 1000)
        )

    def open(self):
        self.filename = self.getTimestampString() + "_" + self.type + ".csv"
        self.file = open(self.filename, "w", newline="")
        self.csv = csv.writer(self.file)

    def close(self):
        if self.csv:
            self.file.close()
            self.csv = 0

    def is_open(self):
        return self.csv != 0

    def write_row(self, row_array):
        if self.csv:
            try:
                self.csv.writerow(row_array)
            except:
                pass

    def write(self, data_matrix):
        if self.csv:
            self.csv.writerows(data_matrix)


def LSPCCallback(data, params):
    buffer, scalefactor = params

    for i in range(int(len(data) / 8)):
        timestamp = (
                int.from_bytes(
                    list(data[8 * i: 8 * i + 4]), byteorder="little", signed=False
                )
                / 100000
        )
        value = (
                int.from_bytes(
                    list(data[8 * i + 4: 8 * i + 8]), byteorder="little", signed=True
                )
                * scalefactor
        )
        buffer.append(value)


previous_time = 0
previous_encoder = 0


def CombinedSampleCallback(data, params):
    global previous_time
    global previous_encoder
    csv_obj = params

    single_sample_package_length = 46
    if (len(data) % single_sample_package_length) == 0:
        num_samples = int(len(data) / single_sample_package_length)

        for i in range(num_samples):
            [
                timestamp,
                PWM_Frequency,
                TimerMax,
                DutyCycleLocation,
                TriggerLocationON,
                TriggerLocationOFF,
                CurrentON,
                CurrentOFF,
                Bemf,
                VbusON,
                VbusOFF,
                Encoder,
            ] = struct.unpack(
                "<IHIIIIfffffi",
                data[
                single_sample_package_length
                * i: single_sample_package_length
                     * (i + 1)
                ],
            )
            timestamp = float(timestamp) / 100000

            if timestamp <= previous_time:
                previous_encoder = Encoder
                previous_time = timestamp
                continue

            previous_encoder = Encoder
            previous_time = timestamp

            if csv_obj:
                csv_obj.write_row(
                    [
                        timestamp,
                        float(PWM_Frequency),
                        float(TimerMax),
                        float(DutyCycleLocation),
                        float(TriggerLocationON),
                        float(TriggerLocationOFF),
                        CurrentON,
                        CurrentOFF,
                        Bemf,
                        VbusON,
                        VbusOFF,
                        float(Encoder),
                    ]
                )


def ControllerDebugCallback(data, params):
    global previous_time
    global previous_encoder
    (
        CurrentSetpoint,
        CurrentFiltered,
        OmegaFiltered,
        IntegralTerm,
        PI,
        Duty,
        csv_obj,
    ) = params

    single_sample_package_length = 36
    if len(data) == single_sample_package_length:
        [
            timestamp,
            current_setpoint,
            current_measurement,
            current_filtered,
            omega_measurement,
            omega_filtered,
            integral,
            PIout,
            duty_set,
        ] = struct.unpack("<Iffffffff", data)
        timestamp = float(timestamp) / 100000

        if timestamp <= previous_time:
            previous_time = timestamp
            return

        # CurrentRaw.append(current_measurement)
        CurrentFiltered.append(current_filtered)
        CurrentSetpoint.append(current_setpoint)

        # OmegaRaw.append(omega_measurement)
        OmegaFiltered.append(omega_filtered)

        IntegralTerm.append(integral)
        PI.append(PIout)
        Duty.append(duty_set)

        previous_time = timestamp

        if csv_obj:
            csv_obj.write_row(
                [
                    timestamp,
                    float(current_setpoint),
                    float(current_measurement),
                    float(current_filtered),
                    float(omega_measurement),
                    float(omega_filtered),
                    float(integral),
                    float(PIout),
                    float(duty_set),
                ]
            )


def SetDutyCycle(params):
    com, dutyCycleField = params
    value = float(dutyCycleField.get())
    if value < -1:
        value = -1
    elif value > 1:
        value = 1

    int_value = value * 1000
    data = int(int_value).to_bytes(length=2, byteorder="little", signed=True)

    print("Setting PWM Duty cycle to {}".format(value))
    com.transmit(0x01, bytearray(data[0:2]))


def SetFrequencies(params):
    com, PWMFrequencyField, SampleFrequencyField = params
    PWMFrequency = int(PWMFrequencyField.get())
    SampleFrequency = int(SampleFrequencyField.get())

    bPWMFrequency = PWMFrequency.to_bytes(length=2, byteorder="little", signed=False)
    bSampleFrequency = SampleFrequency.to_bytes(
        length=2, byteorder="little", signed=False
    )

    print(
        "Setting PWM Frequency to {} and Sample frequency to {}".format(
            PWMFrequency, SampleFrequency
        )
    )
    com.transmit(0x02, bytearray(bPWMFrequency + bSampleFrequency))


def SetAveraging(params):
    com, AveragingCountField = params
    AveragingCount = int(AveragingCountField.get())

    bAveragingCount = AveragingCount.to_bytes(
        length=2, byteorder="little", signed=False
    )

    print("Setting Averaging number of samples to {}".format(AveragingCount))
    com.transmit(0x05, bytearray(bAveragingCount))


def SetLimits(params):
    minField, maxField, setLimFun = params

    if setLimFun:
        setLimFun(float(minField.get()), float(maxField.get()))


def SetCurrent(params):
    com, currentSetpointField = params
    value = float(currentSetpointField.get())

    int_value = value * 1000
    data = int(int_value).to_bytes(length=2, byteorder="little", signed=True)

    print("Setting Current Setpoint to {}".format(value))
    com.transmit(0x0A, bytearray(data[0:2]))


def SetCurrentKpTi(params):
    com, currentKpField, currentTiField = params
    Kp = float(currentKpField.get())
    Ti = float(currentTiField.get())

    Kp_value = Kp * 100
    Ti_value = Ti * 100
    Kp_data = int(Kp_value).to_bytes(length=2, byteorder="little", signed=True)
    Ti_data = int(Ti_value).to_bytes(length=2, byteorder="little", signed=True)

    print("Setting Current Kp gain to {}".format(Kp))
    print("Setting Current Ti gain to {}".format(Ti))
    com.transmit(0x0B, bytearray(Kp_data[0:2]) + bytearray(Ti_data[0:2]))


def ToggleLog(params):
    btn, com, logger = params

    if logger:
        if not logger.is_open():
            com.flush()
            logger.open()
            btn["text"] = "Disable Logger"
        else:
            logger.close()
            btn["text"] = "Enable Logger"


def ChangeMode(params):
    com, btn = params

    if "Brake Mode" in btn["text"]:
        btn["text"] = "Coast Mode"
        bMode = 1
    else:
        btn["text"] = "Brake Mode"
        bMode = 0

    com.transmit(0x03, bytearray([bMode]))


def ChangeActiveInactive(params):
    com, btn = params

    if "Active" in btn["text"]:
        btn["text"] = "Inactive"
        bActive = 0
    else:
        btn["text"] = "Active"
        bActive = 1

    com.transmit(0x04, bytearray([bActive]))


def StartCurrentSweep(params):
    com = params
    com.transmit(0x0C, bytearray())


def StartDutySweep(params):
    com = params
    com.transmit(0x07, bytearray())


def StartSampleLocationSweep(params):
    com = params
    com.transmit(0x09, bytearray())


def CalibrationCurrent(params):
    com, add_btn, current_btn, vbus_btn, bemf_btn = params

    if "disabled" in add_btn["state"]:
        add_btn["state"] = "normal"
        vbus_btn["state"] = "disabled"
        bemf_btn["state"] = "disabled"
        com.transmit(0x08, bytearray([0x01]))
    else:
        add_btn["state"] = "disabled"
        vbus_btn["state"] = "normal"
        bemf_btn["state"] = "normal"
        com.transmit(0x08, bytearray([0x00]))


def CalibrationVbus(params):
    com, add_btn, current_btn, vbus_btn, bemf_btn = params

    if "disabled" in add_btn["state"]:
        add_btn["state"] = "normal"
        current_btn["state"] = "disabled"
        bemf_btn["state"] = "disabled"
        com.transmit(0x08, bytearray([0x02]))
    else:
        add_btn["state"] = "disabled"
        current_btn["state"] = "normal"
        bemf_btn["state"] = "normal"
        com.transmit(0x08, bytearray([0x00]))


def CalibrationBemf(params):
    com, add_btn, current_btn, vbus_btn, bemf_btn = params

    if "normal" in current_btn["state"]:
        current_btn["state"] = "disabled"
        vbus_btn["state"] = "disabled"
        com.transmit(0x08, bytearray([0x03]))
    else:
        current_btn["state"] = "normal"
        vbus_btn["state"] = "normal"
        com.transmit(0x08, bytearray([0x00]))


def AddCalibrationMeasurement(params):
    com, valueField = params
    value = float(valueField.get())

    int_value = value * 1000
    data = int(int_value).to_bytes(length=2, byteorder="little", signed=True)

    com.transmit(0x08, bytearray([0x03]) + bytearray(data[0:2]))

def RequestCPULoad(params):
    com = params
    com.transmit(0x0E, bytearray([0x00])) # disable continuous sending
    com.transmit(0x0F, bytearray())

def CPU_Load_Print(data, params):
    output = data.decode("utf-8").rstrip()
    print(output)


def DebugPrint(data, params):
    timestamp = datetime.now().strftime("%H:%M:%S") + ".{0:03d}".format(
        round(datetime.now().microsecond / 1000)
    )
    print("[" + timestamp + "] " + data.decode("utf-8"))


def signal_handler(com, root, signal, frame):
    # Re-enabled continuous CPU load messages
    com.transmit(0x0E, bytearray([0x01]))
    while not com.tx_queue.empty():  # wait for message to be transmitted
        time.sleep(0.050)
    com.close()

    root.destroy()
    root.quit()


def main():
    # Spawn GUI
    root = Tk.Tk()
    root.title("Motor Driver GUI")
    main = MainWindow(root)

    # Add plots
    # CurrentRaw = main.liveplot.addGraph('Current', 'A')
    CurrentFiltered = main.liveplot.addGraph("Current Filtered", "A")
    CurrentSetpoint = main.liveplot.addGraph("Current Setpoint", "A")
    # OmegaRaw = main.liveplot.addGraph('Omega', 'rad/s')
    OmegaFiltered = main.liveplot.addGraph("Omega Filtered", "rad/s")
    Integral = main.liveplot.addGraph("Integral", "V")
    PIout = main.liveplot.addGraph("PI", "V")
    DutyCycle = main.liveplot.addGraph("Duty cycle")
    main.liveplot.setYlim(-0.2, 0.8)

    # Connect to CAN bus
    ports = lspc.list_serial_ports()
    print(ports)
    com = lspc.LSPC(ports[0])
    com.open(1612800)

    # Disable continuous CPU load messages
    #com.transmit(0x0E, bytearray([0x00]))

    # CSV files
    logger = BufferedCSVwriter("controller")

    # Link messages to receiver threads
    # com.registerCallback(0x04, CombinedSampleCallback, (logger))
    com.registerCallback(0x05, ControllerDebugCallback,
        (
            CurrentSetpoint,
            CurrentFiltered,
            OmegaFiltered,
            Integral,
            PIout,
            DutyCycle,
            logger,
        ),
    )
    com.registerCallback(0xE1, CPU_Load_Print, ())  # CPU Load
    com.registerCallback(0xFF, DebugPrint, ())  # Debug print

    # Register button press
    main.toolbar.registerCallback("set_dutycycle", SetDutyCycle, (com, main.toolbar.DutyCycle))
    main.toolbar.registerCallback("set_frequencies", SetFrequencies,
        (com, main.toolbar.PWMFrequency, main.toolbar.SampleFrequency),
    )
    main.toolbar.registerCallback("set_averaging", SetAveraging, (com, main.toolbar.AveragingCount))
    main.toolbar.registerCallback("set_current", SetCurrent, (com, main.toolbar.CurrentSetpoint))
    main.toolbar.registerCallback("set_kpti", SetCurrentKpTi,
        (com, main.toolbar.CurrentKp, main.toolbar.CurrentTi),
    )
    main.toolbar.registerCallback("set_limits", SetLimits,
        (main.toolbar.YlimMin, main.toolbar.YlimMax, main.liveplot.setYlim)
    )
    main.toolbar.registerCallback("toggle_logger", ToggleLog, (main.toolbar.ToggleLogBtn, com, logger))
    main.toolbar.registerCallback("start_current_sweep", StartCurrentSweep, (com))
    main.toolbar.registerCallback("start_duty_sweep", StartDutySweep, (com))
    main.toolbar.registerCallback("start_sample_location_sweep", StartSampleLocationSweep, (com))
    main.toolbar.registerCallback("calibration_current", CalibrationCurrent,
        (
            com,
            main.toolbar.AddCalibrationMeasurementButton,
            main.toolbar.CurrentCalibrationButton,
            main.toolbar.VbusCalibrationButton,
            main.toolbar.BemfCalibrationButton,
        ),
    )
    main.toolbar.registerCallback("calibration_vbus", CalibrationVbus,
        (
            com,
            main.toolbar.AddCalibrationMeasurementButton,
            main.toolbar.CurrentCalibrationButton,
            main.toolbar.VbusCalibrationButton,
            main.toolbar.BemfCalibrationButton,
        ),
    )
    main.toolbar.registerCallback("calibration_bemf", CalibrationBemf,
        (
            com,
            main.toolbar.AddCalibrationMeasurementButton,
            main.toolbar.CurrentCalibrationButton,
            main.toolbar.VbusCalibrationButton,
            main.toolbar.BemfCalibrationButton,
        ),
    )
    main.toolbar.registerCallback("calibration_add_measurement", AddCalibrationMeasurement,
        (com, main.toolbar.CalibrationMeasurement),
    )
    main.toolbar.registerCallback('print_cpuload', RequestCPULoad, (com))

    signal.signal(signal.SIGINT, partial(signal_handler, com, root))

    # Main Tk (window) loop
    root.mainloop()

    # Finished (user exit)
    if (com.isOpen):
        # Re-enabled continuous CPU load messages
        com.transmit(0x0E, bytearray([0x01]))
        while not com.tx_queue.empty():  # wait for message to be transmitted
            time.sleep(0.050)
        com.close()
    print('Exiting...')


if __name__ == "__main__":
    main()
