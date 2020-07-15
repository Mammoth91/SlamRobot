#!/usr/bin/env python3

'''
rpslam.py : BreezySLAM Python with SLAMTECH RP A1 Lidar
                 
Copyright (C) 2018 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 20
LIDAR_DEVICE            = '/dev/ttyUSB0'
HOST_IP      = "127.0.0.1"
HOST_PORT    = 6969
BUFFER_SIZE  = 1024

# Ideally we could use all 250 or so samples that the RPLidar delivers in one 
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES   = 140

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from roboviz import MapVisualizer
import socket

if __name__ == '__main__':
    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None
    
    # Create UDP Socket
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPServerSocket.bind((HOST_IP, HOST_PORT))
	    
    while True:
        # Extract distances and angles from UDP messages
        distances = []
        angles    = []
        
        while True:
            bytesAddressPair = UDPServerSocket.recvfrom(BUFFER_SIZE)
            message = bytesAddressPair[0]
			
            if str(message).find("&end&") != -1:
                break
            else:
                distances.append(float(str(message).split(" ")[0].replace("'","").replace("b","")))
                angles.append(float(str(message).split(" ")[1].replace("'","").replace("b","")))
        
        # Get the longest distance
        #longestDistance = 0.0
        #longestAngle    = 0.0
        #for i in range(len(distances)):
        #    if distances[i] > longestDistance:
        #        longestDistance = distances[i]
        #        longestAngle    = angles[i]
        

        # Debug
        #print(quality[0])
        #print(distances[0])
        #print(angles[0])
        #print(longestDistance)
        #print(longestAngle)
        #print(str(len(distances)) + " " + str(len(angles)) + " " + str(len(angles)) + " " + str(len(items)))

        # Update SLAM with current Lidar scan
        slam.update(distances, scan_angles_degrees=angles)
        previous_distances = distances.copy()
        previous_angles    = angles.copy()


        # Get current robot position
        x, y, theta = slam.getpos()

        # Get current map bytes as grayscale
        slam.getmap(mapbytes)

        # Display map and robot pose, exiting gracefully if user closes it
        if not viz.display(x/1000., y/1000., theta, mapbytes):
            exit(0)
