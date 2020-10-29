#!/usr/bin/env python
from os import system
import subprocess
import sys
from sys import argv
import time
import argparse
import json

if __name__ == '__main__':
    # Parse command line arguments:
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', type=int, default=1, help='Number of vehicles to simulate')
    parser.add_argument('-type', type=str, default="ArduPlane", help='Type of vehicle. NOTE: If set to ArduCopter, the jsbVehicle flag is ignored')
    parser.add_argument('-jsbVehicle', type=str, default="Talon", help='Type of JSBSim vehicle model to load')
    parser.add_argument('-console', type=bool, default=False, help='Display ardupilot console')
    args = parser.parse_args()

    system('tmux new -d -s vehicles')
    for i in range(args.n):        
        vehicleID = i+1
        print("____________ Launch {} with Vehicle ID = {} ____________".format(args.type, vehicleID))

        time.sleep(3)
        windowName = '{}_{}'.format(args.type, vehicleID)
        system('tmux new-window -n {}'.format(windowName))
        vehicleType = args.type
        jsbsimVehicle = args.jsbVehicle
        outputIP = "127.0.0.1"
        outputPort = 14550 + vehicleID
        lat = -35.3632621 + vehicleID/10
        lng = 149.1652374 + vehicleID/100
        alt = 0.0
        heading = 0.0

        if args.console == True:
            if args.type == "ArduCopter":
                system('tmux send-keys -t {} "sim_vehicle.py --custom-location={},{},{},{} --out=udp:{}:{} --instance={} --sysid={} --vehicle={} --console" ENTER'.format(windowName, lat, lng, alt, heading, outputIP, outputPort, vehicleID, vehicleID, vehicleType))
            else:
                system('tmux send-keys -t {} "sim_vehicle.py --custom-location={},{},{},{} --out=udp:{}:{} --instance={} --sysid={} --vehicle={} --console -f jsbsim:{}" ENTER'.format(windowName, lat, lng, alt, heading, outputIP, outputPort, vehicleID, vehicleID, vehicleType, jsbsimVehicle))
        else:
            if args.type == "ArduCopter":
                system('tmux send-keys -t {} "sim_vehicle.py --custom-location={},{},{},{} --out=udp:{}:{} --instance={} --sysid={} --vehicle={}" ENTER'.format(windowName, lat, lng, alt, heading, outputIP, outputPort, vehicleID, vehicleID, vehicleType))
            else:
                system('tmux send-keys -t {} "sim_vehicle.py --custom-location={},{},{},{} --out=udp:{}:{} --instance={} --sysid={} --vehicle={} -f jsbsim:{}" ENTER'.format(windowName, lat, lng, alt, heading, outputIP, outputPort, vehicleID, vehicleID, vehicleType, jsbsimVehicle))


    # Attach to tmux:
    system('tmux a')
