# System, threading and others
import collections  # for deque = circular buffer
import os
import time
import tkinter as Tk
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

from CANBus import CANBus


class Graph:
    def __init__(self, ax, bufferLength, index, name, unit):
        self.name = name
        colors = ["r-", "g-", "b-", "y-", "c-"]
        self.plot = ax.plot([], [], colors[index], label=name)[0]
        self.label = ax.text(0.80, 0.90 - index * 0.05, "", transform=ax.transAxes)
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
        self.ax.set_title("Analog readings")
        self.ax.set_xlabel("Sample index [k]")
        self.ax.set_ylabel("Reading [V/10 or A]")

        # Assign figure to drawing/window canvas
        canvas = FigureCanvasTk(self.figure, self)
        canvas.draw()
        canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=True)

        self.graphs = []

        # Configure animation (refresh/redraw) of figure - refresh every 1 ms
        self.anim = animation.FuncAnimation(
            self.figure, self.__updatePlot__, interval=50
        )  # fargs has to be a tuple, e.g. fargs=(graph,graphText)

    def __updatePlot__(self, frame):
        for graph in self.graphs:
            graph.redraw()

    def addGraph(self, name, unit):
        graph = Graph(self.ax, self.bufferLength, len(self.graphs), name, unit)
        self.graphs.append(graph)
        return graph.buffer

    def setYlim(self, min, max):
        self.ax.set_ylim([min, max])


class Toolbar(Tk.Frame):
    def __init__(self, parent):
        Tk.Frame.__init__(self, parent)

        self.callbacks = {}

        # Construct toolbox items
        lbl1 = Tk.Label(self, text="PWM")
        lbl1.pack(padx=5, pady=5)
        self.PWM = Tk.Entry(self)
        self.PWM.insert(0, "0")  # (index, string)
        self.PWM.pack(padx=5)
        SendButton = Tk.Button(
            self, text="Send", command=lambda: self.__btnHandler__("send")
        )
        SendButton.pack(padx=5)

    def __btnHandler__(self, callbackName):
        print("Send button pressed")
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


class PlotUpdaterThread:
    def __init__(self, buffer, mean, sigma):
        self.shouldRun = True
        self.buffer = buffer
        self.mean = mean
        self.sigma = sigma

        thread = Thread(target=self.__task__)
        thread.start()

    def __task__(self):
        while self.shouldRun:
            self.buffer.append(np.random.normal(self.mean, self.sigma))
            time.sleep(0.050)


def CANCallback(data, params):
    buffer, scalefactor = params
    timestamp = (
        int.from_bytes(list(data[0:4]), byteorder="little", signed=False) / 10000
    )
    volt = int.from_bytes(list(data[4:6]), byteorder="little", signed=False) / 1000
    output = volt * scalefactor
    buffer.append(output)


def PWMButtonPress(params):
    can, entryField = params
    value = int(entryField.get())
    if value < 0:
        value = 0
    elif value > 255:
        value = 255

    print("Setting PWM to {}".format(value))
    can.transmit("0x101", [0, value])


def main():
    # Spawn GUI
    root = Tk.Tk()
    root.title("Motor Driver GUI")
    main = MainWindow(root)

    # Add plots
    CS = main.liveplot.addGraph("CS", "A")
    CS_VNH = main.liveplot.addGraph("CS VNH", "A")
    VBAT = main.liveplot.addGraph("VBAT", "V")
    main.liveplot.setYlim(0, 1)

    # Connect to CAN bus
    can = CANBus()
    can.open()

    # Link messages to receiver threads
    can.registerCallback("0x101", CANCallback, (CS, 2.5))
    can.registerCallback("0x102", CANCallback, (CS_VNH, 1.985))
    can.registerCallback("0x103", CANCallback, (VBAT, 0.1 * 1.0 / 0.108))

    # Register button press
    main.toolbar.registerCallback("send", PWMButtonPress, (can, main.toolbar.PWM))

    # Main Tk (window) loop
    root.mainloop()

    # Finished (user exit)
    can.close()
    print("Exiting...")


if __name__ == "__main__":
    main()
