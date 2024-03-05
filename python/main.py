#!/usr/bin/env python3

import ntcore
import time
import csv
from pathlib import Path

SLEEP_TIME = 0.05
CSV_FOLDER = "logs"


class LogLevels:
    SILENT = 0
    INFO = 1
    VERBOSE = 2


class Logger:
    def __init__(self, fileName, tableName="SmartDashboard", timestamps=True, logging=1):
        inst = ntcore.NetworkTableInstance.getDefault()
        self.table = inst.getTable(tableName)
        inst.startClient4("very very very good logging zzzzzzzzzzzz")
        inst.setServerTeam(7525)
        inst.startDSClient()
        self.subscriptions = []
        p = Path(CSV_FOLDER).joinpath(fileName)
        self.file = csv.writer(open(p, "w", newline=""))
        self.timestamps = timestamps
        if self.logging >= 1: 
            print(f"{time.strftime('%H:%M:%S')}: Connected to table {tableName}")
        self.startTime = time.time()

    def logString(self, stringName: str):  # grrrr camel case
        sub = self.table.getStringTopic(stringName).subscribe("None")
        self.subscriptions.append(sub)
        if self.logging >= 1: 
            print(f"{time.strftime('%H:%M:%S')}: Connected to table {self.table}")

    def logDouble(self, stringName: str):  # grrrr camel case
        sub = self.table.getDoubleTopic(stringName).subscribe(0)
        self.subscriptions.append(sub)
        if self.logging >= 1: 
            print(f"{time.strftime('%H:%M:%S')}: Subscripted to {stringName}")

    def initializeCsv(self):
        header = []
        if self.timestamps:
            header.append("time")
        for sub in self.subscriptions:
            header.append(sub.getTopic().getName())
        self.file.writerow(header)
        if self.logging >= 1: 
            print(f"{time.strftime('%H:%M:%S')}: Added CSV headers {header}")

    def addToCSV(self):
        vals = []
        for sub in self.subscriptions:
            vals.append(sub.get())
        if self.timestamps:
            vals.append(time.time() - self.startTime)
        self.file.writerow(vals)
        if self.logging >= 2:
            print(f"{time.strftime('%H:%M:%S')}: Wrote row {vals}")


def logSmartDashboard():
    log = Logger(fileName=f"{time.strftime('%Y-%m-%d-%H-%M')}-smartdashboard.csv")

    # so ya how you use this is you subscribe to what you want outside of the loop and
    # then do whatever printing/saving/wtv inside the loop

    # log state strings
    log.logString("Shooting States")
    log.logString("Manager State")
    log.logString("Drive State")
    log.logString("Climber State")
    log.logString("Current state of INTAKE:")  # why is it named like this.
    log.logString("Currently selected autonomous")

    # log climber stuff
    log.logDouble("Left Climber Current")
    log.logDouble("Left Encoder Position")
    log.logDouble("Left Encoder Setpoint")
    log.logDouble("Right Climber Current")
    log.logDouble("Right Encoder Position")
    log.logDouble("Right Encoder Setpoint")

    # log intake stuff
    log.logDouble("Intake motor current")
    log.logDouble("intake motor position")
    log.logDouble("intake motor setpoint")
    log.logDouble("pivot motor position")
    log.logDouble("pivot motor setpoint")

    # i think this is the shooter? yall need better names
    log.logDouble("Motor 1 velocity")
    log.logDouble("Motor 2 velocity")

    # log drive stuff
    log.logDouble("Robot Velocity")
    log.logDouble("Acceleration")

    # log path stuff
    log.logString("Path Chooser/active")
    log.logString("Path Chooser/selected")

    return log


if __name__ == "__main__":
    smartDash = logSmartDashboard()
    smartDash.initializeCsv()

    while True:
        time.sleep(SLEEP_TIME)  # so it doesnt kill your disk/memory
		smartDash.addToCSV()
