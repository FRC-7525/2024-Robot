#!/usr/bin/env python3

import ntcore
import time
import csv
from pathlib import Path

#oh my god i hate python WHERE ARE THE SEMICOLONS
SLEEP_TIME = 0.05
CSV_FOLDER = "logs"

class Logger:
    def __init__(self, fileName, timestamps=True):
        inst = ntcore.NetworkTableInstance.getDefault()
        self.table = inst.getTable("SmartDashboard")
        inst.startClient4("very good logging zzzzzzzzzzzz") #youll never take me alive
        inst.setServerTeam(7525)
        inst.startDSClient()
        self.subscriptions = [] #ok i have to admit this is easier than Vec::<Type>::new()
        p = Path("..").joinpath(CSV_FOLDER).joinpath(fileName) #this is bad but wtv
        self.file = csv.writer(open(p, "w", newline=''))
        self.timestamps = timestamps

    def logString(self, string_name: str): # grrrr camel case
        sub = self.table.getStringTopic(string_name).subscribe("None")
        self.subscriptions.append(sub)

    def logDouble(self, string_name: str): # grrrr camel case
        sub = self.table.getDoubleTopic(string_name).subscribe(0)
        self.subscriptions.append(sub)

    def initializeCsv(self):
        header = []
        for sub in self.subscriptions:
            header.append(sub.getTopic().getName())
        if self.timestamps:
            header.append("time")
        self.file.writerow(header)

    def addToCSV(self):
        vals = []
        for sub in self.subscriptions:
            vals.append(sub.get())   
        if self.timestamps:
            vals.append(time.strftime(f"%H:%M:%S"))
        self.file.writerow(vals)

if __name__ == "__main__":
    log = Logger(fileName="smartdashboard.csv")

    #so ya how you use this is you subscribe to what you want outside of the loop and 
    #then do whatever printing/saving/wtv inside the loop

    #log state strings
    log.logString("Shooting States")
    log.logString("Manager State")
    log.logString("Drive State")
    log.logString("Climber State")
    log.logString("Current state of INTAKE:") #why is it named like this.
    log.logString("Currently selected autonomous")
    
    #log climber stuff
    log.logDouble("Left Climber Current")
    log.logDouble("Left Encoder Position")
    log.logDouble("Left Encoder Setpoint")
    log.logDouble("Right Climber Current")
    log.logDouble("Right Encoder Position")
    log.logDouble("Right Encoder Setpoint")

    #log intake stuff
    log.logDouble("Intake motor current")
    log.logDouble("intake motor position")
    log.logDouble("intake motor setpoint")
    log.logDouble("pivot motor position")
    log.logDouble("pivot motor setpoint")

    #i think this is the shooter? yall need better names
    log.logDouble("Motor 1 velocity")
    log.logDouble("Motor 2 velocity")

    #log drive stuff
    log.logDouble("Robot Velocity")
    log.logDouble("Acceleration")

    #log path stuff
    log.logString("Path Chooser/active")
    log.logString("Path Chooser/selected")

    while True:
        time.sleep(SLEEP_TIME) #so it doesnt kill your disk/memory 
        log.addToCSV()