#!/usr/bin/env python2

#                USAGE
#
#
# - Import the RobotVisualiser.py file
# - Create a new instance of RobotVisualiser
#   This will open the visual window
# - Whenever the model updates, call .setRobotModel(robotModel)
# - To close the window, call .close()

import RobotModel

import Tkinter as tk
from multiprocessing import Process, Pipe

# Some additions to the RobotModel
class VisualiserModel(RobotModel.RobotModel):
    shouldClose = False # Track whether the window should close


# Window the Visualiser displays in
class Window(tk.Frame):
    def __init__(self, conn, master=None):
        self.conn = conn # make the pipe connector available to other methods

        tk.Frame.__init__(self, master)
        self.grid()

        self.createCanvas()

        self.after(2000, self.update)  # set task to execute in 2000ms

    # Creates the canvas and creates the objects on it
    def createCanvas(self):
        self.w = tk.Canvas(self, width=200, height=100)
        self.w.pack()
        self.textLeft = self.w.create_text(50, 80, text="Left")
        self.textRight = self.w.create_text(150, 80, text="Right")
        self.textClaw = self.w.create_text(100, 10, text="Claw")
        self.textReed = self.w.create_text(100, 30, text="Reed")

    def update(self):
        # read from the pipe until the most recent is in self.robotModel
        while self.conn.poll():
            self.robotModel = self.conn.recv()

        # modify the canvas objects to reflect new model
        self.w.itemconfig(self.textLeft, text=self.robotModel.speedLeft)
        self.w.itemconfig(self.textRight, text=self.robotModel.speedRight)
        self.w.itemconfig(self.textClaw, text=self.robotModel.clawClosed)
        self.w.itemconfig(self.textReed, text=self.robotModel.reedSwitch)

        if self.robotModel.shouldClose:
            self.master.destroy()
        else:
            self.after(100, self.update) # set itself as a task in 100ms

class RobotVisualiser:
    def __init__(self):
        self.parent_conn, child_conn = Pipe() # create pipe

        self.visualiserModel = VisualiserModel()
        self.parent_conn.send(self.visualiserModel) # send the initial model

        self.p = Process(target=self.__createWindow, args=(child_conn,)) # create another thread to create the window
        self.p.start() # start the thread

    def __createWindow(self, conn):
        self.window = Window(conn)
        self.window.mainloop() # start the main loop of the window (will continue until window is closed)

    def __update(self):
        self.parent_conn.send(self.visualiserModel) # send the current model

    def setRobotModel(self, robotModel):
        # set attributes of the visualiserModel to the attributes of the RobotModel received
        self.visualiserModel.speedLeft = robotModel.speedLeft
        self.visualiserModel.speedRight = robotModel.speedRight
        self.visualiserModel.clawClosed = robotModel.clawClosed
        self.visualiserModel.reedSwitch = robotModel.reedSwitch
        self.__update()

    def setReedSwitch(self, condition):
        self.visualiserModel.reedSwitch = condition
        self.__update()

    def setSpeedLeft(self, speed):
        self.visualiserModel.speedLeft = speed
        self.__update()

    def setSpeedRight(self, speed):
        self.visualiserModel.speedRight = speed
        self.__update()

    def setClawClosed(self, condition):
        self.visualiserModel.clawClosed = condition
        self.__update()

    def close(self):
        self.visualiserModel.shouldClose = True
        self.__update()
        self.p.join()  # wait for the window to close and thread finish executing
