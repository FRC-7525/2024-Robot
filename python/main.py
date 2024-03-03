#!/usr/bin/env python3

import ntcore
import time
import csv
from pathlib import Path

#oh my god i hate python WHERE ARE THE SEMICOLONS
SLEEP_TIME = 0.05
CSV_FOLDER = "logs"
CSV_FILE_NAME = "log.csv"

class Logger:
    def __init__(self):
        inst = ntcore.NetworkTableInstance.getDefault()
        self.table = inst.getTable("SmartDashboard")
        inst.startClient4("very good logging zzzzzzzzzzzz") #youll never take me alive
        inst.setServerTeam(7525)
        inst.startDSClient()
        self.subscriptions = [] #ok i have to admit this is easier than Vec::<Type>::new()
        p = Path(CSV_FOLDER).absolute().joinpath("..").joinpath(CSV_FILE_NAME) #this is bad but wtv
        self.file = csv.writer(open(p, "w", newline=''))

    def logString(self, string_name: str): # grrrr camel case
        sub = self.table.getStringTopic(string_name).subscribe("None")
        self.subscriptions.append(sub)

    def logDouble(self, string_name: str): # grrrr camel case
        sub = self.table.getDoubleTopic(string_name).subscribe(0)
        self.subscriptions.append(sub)

    def printLogs(self):
        vals = []
        for sub in self.subscriptions: #i hate this.
            vals.append(sub.get())
        print(vals)

    def initializeCsv(self):
        header = []
        for sub in self.subscriptions:
            header.append(sub.getTopic().getName())
        self.file.writerow(header)

    def addToCSV(self):
        vals = []
        for sub in self.subscriptions:
            vals.append(sub.get())   
        self.file.writerow(vals)

if __name__ == "__main__":
    log = Logger()

    #so ya how you use this is you subscribe to what you want outside of the loop and 
    #then do whatever printing/saving/wtv inside the loop

    #log state strings
    log.logString("Shooting States")
    log.logString("Manager State")
    log.logString("Drive State")
    log.logString("Climber State")

    log.initializeCsv()

    while True:
        time.sleep(SLEEP_TIME) #so it doesnt kill your disk/memory 
        log.addToCSV()