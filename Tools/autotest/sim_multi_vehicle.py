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
    parser.add_argument('fp', type=str, help='Relative file path to the launch script JSON configuration file. (e.g.: ./config_1_vehicle.json')
    config = parser.parse_args()
    with open(config.fp, 'r') as in_file:
        config = json.load(in_file)

    vehicles = config['vehicles']

    ## Options for {vehicle_type}: ArduCopter|AntennaTracker|APMrover2|ArduSub|ArduPlane
    ## Options for {frame_type} depend on vehicle type. For example, for ArduCopter: octa-quad|tri|singlecopter|gazebo-iris|calibration|hexa|heli|+|heli-compound|dodeca-hexa|heli-dual|coaxcopter|X|quad|y6|IrisRos|octa

    # Loop over vehicle ID/sensor array to extend the command to launch each vehicle according to their type
    if len(vehicles.items()) > 0:
		system('tmux new -d -s vehicles')
		for key, value in vehicles.items():
			print("____________ Launch quad with Vehicle ID = {} ____________".format(key))
			location = value["position"]

			if value["simulated"] == True:
				time.sleep(1)
				windowName = '{}_{}'.format(value["type"], key)
				system('tmux new-window -n {}'.format(windowName))
				if value["console"] == True:
					system('tmux send-keys -t {} "cd ardupilot/ArduCopter; sim_vehicle.py --custom-location={},{},{},{} --out=udp:{}:{} --instance={} --vehicle={} --frame={} --console" ENTER'.format(windowName, location["lat"], location["lng"], location["alt_msl"], location["heading_deg"], value["outputIP"], value["outputPort"], key, value["type"], value["frame"]))
				else:
					system('tmux send-keys -t {} "cd ardupilot/ArduCopter; sim_vehicle.py --custom-location={},{},{},{} --out=udp:{}:{} --instance={} --vehicle={} --frame={}" ENTER'.format(windowName, location["lat"], location["lng"], location["alt_msl"], location["heading_deg"], value["outputIP"], value["outputPort"], key, value["type"], value["frame"]))


    # Attach to tmux:
    system('tmux a')
