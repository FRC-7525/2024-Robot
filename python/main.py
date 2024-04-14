#!/usr/bin/env python3

import ntcore
import time
import csv
from pathlib import Path

SLEEP_TIME = 0.1
CSV_FOLDER = "C:\\Code\\2024-Robot\\logs"

class LogLevels:
    SILENT = 0
    INFO = 1
    VERBOSE = 2

class Logger:
    def __init__(self, file_name, table_name="SmartDashboard", timestamps=True, logging=LogLevels.INFO):
        inst = ntcore.NetworkTableInstance.getDefault()
        self.table = inst.getTable(table_name)
        inst.startClient4("very very very good logging zzzzzzzzzzzz")
        inst.setServerTeam(7525)
        inst.startDSClient()
        self.subscriptions = []
        p = Path(CSV_FOLDER).joinpath(file_name)
        self.file = csv.writer(open(p, "w", newline=""))
        self.timestamps = timestamps

        self.logging = logging
        
        if self.logging >= 1: 
            print(f"{time.strftime('%H:%M:%S')}: Connected to table {table_name}")
        self.start_time = time.time()

    def log_string(self, string_name: str):
        sub = self.table.getStringTopic(string_name).subscribe("None")
        self.subscriptions.append(sub)
        if self.logging >= LogLevels.INFO: 
            print(f"{time.strftime('%H:%M:%S')}: Connected to table {self.table}")

    def log_double(self, string_name: str):
        sub = self.table.getDoubleTopic(string_name).subscribe(0)
        self.subscriptions.append(sub)
        if self.logging >= LogLevels.INFO: 
            print(f"{time.strftime('%H:%M:%S')}: Subscripted to {string_name}")

    def log_double_array(self, array_name: str):
        sub = self.table.getDoubleArrayTopic(array_name).subscribe([0.0, 0.0, 0.0])
        self.subscriptions.append(sub)
        if self.logging >= LogLevels.INFO: 
            print(f"{time.strftime('%H:%M:%S')}: Subscripted to {array_name}")
    
    def log_boolean(self, bool_name: str):
        sub = self.table.getBooleanTopic(bool_name).subscribe(True)
        self.subscriptions.append(sub)
        if self.logging >= LogLevels.INFO:
            print(f"{time.strftime('%H:%M:%S')}: Subscripted to {bool_name}")

    def initialize_csv(self):
        header = []
        if self.timestamps:
            header.append("time")
        for sub in self.subscriptions:
            header.append(sub.getTopic().getName().split("/SmartDashboard/")[1])
        self.file.writerow(header)
        if self.logging >= LogLevels.INFO: 
            print(f"{time.strftime('%H:%M:%S')}: Added CSV headers {header}")

    def add_to_csv(self):
        vals = []
        if self.timestamps:
            vals.append(time.time() - self.start_time) # aiden wrote this!!1!1
        for sub in self.subscriptions:
            vals.append(sub.get())
        self.file.writerow(vals)
        if self.logging >= LogLevels.VERBOSE:
            print(f"{time.strftime('%H:%M:%S')}: Wrote row {vals}")


def log_smart_dashboard():
    log = Logger(file_name=f"{time.strftime('%Y-%m-%d-%H-%M')}-smartdashboard.csv")

    # so ya how you use this is you subscribe to what you want outside of the loop and
    # then do whatever printing/saving/wtv inside the loop

    # log state strings
    log.log_string("Shooting States")
    log.log_string("Manager State")
    log.log_string("Drive State")
    log.log_string("Climber State")
    log.log_string("Intake State")
    log.log_string("Currently selected autonomous")
    log.log_string("Match State")

    # log climber stuff
    log.log_double("Left Climber Current")
    log.log_double("Left Encoder Position")
    log.log_double("Left Encoder Setpoint")
    #log.log_double("Right Climber Current")
    #log.log_double("Right Encoder Position")
    #log.log_double("Right Encoder Setpoint")
    log.log_boolean("Climb In Progress")

    # log intake stuff
    log.log_double("Intake motor current")
    log.log_double("intake motor position")
    log.log_double("intake motor setpoint")
    log.log_double("pivot motor position")
    log.log_double("pivot motor setpoint")

    # i think this is the shooter? yall need better names
    log.log_double("Shooter Motor 1 velocity")
    log.log_double("Shooter Motor 2 velocity")

    # log drive stuff
    log.log_double("Robot Velocity")
    log.log_double("Acceleration")

    # robot position
    log.log_double("Robot X")
    log.log_double("Robot Y")
    log.log_double("Robot Theta (deg)")

    # front camera vision pose
    log.log_double_array("Front Pose")
    log.log_double_array("Side Pose")

    # battery voltage
    log.log_double("Battery Voltage")
    log.log_double("Total Current")

    return log

if __name__ == "__main__":
    smartDash = log_smart_dashboard()
    smartDash.initialize_csv()

    while True:
        time.sleep(SLEEP_TIME)  # so it doesnt kill your disk/memory
        smartDash.add_to_csv()
