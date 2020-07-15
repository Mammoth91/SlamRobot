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

LIDAR_DEVICE = '/dev/ttyUSB0'
HOST_IP      = "127.0.0.1"
HOST_PORT    = 6969

from rplidar import RPLidar as Lidar
import socket


if __name__ == '__main__':

    # Connect to Lidar unit
    lidar = Lidar(LIDAR_DEVICE)
    print(lidar.get_health())
    print(lidar.get_info())

    # Create an iterator to collect scan data from the RPLidar
    iterator = lidar.iter_scans()
    
    # Create UDP Socket
    serverAddressPort   = (HOST_IP, HOST_PORT)
    bufferSize          = 1024
    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None

    # First scan is crap, so ignore it
    next(iterator)

    while True:

        # Extract (quality, angle, distance) triples from current scan
        items = [item for item in next(iterator)]

        # Extract distances and angles from triples
        distances = [item[2] for item in items]
        angles    = [item[1] for item in items]
        quality   = [item[0] for item in items]
        
        #Send Lidar Data To Host
        for i in range(len(distances)):
            bytesToSend = str.encode(str(distances[i]) + " " + str(angles[i]))
            UDPClientSocket.sendto(bytesToSend, serverAddressPort)
        bytesToSend = str.encode("&end&")
        UDPClientSocket.sendto(bytesToSend, serverAddressPort)
	
        # Debug
        #print(quality[0])
        #print(distances[0])
        #print(angles[0])
        #print(str(len(distances)) + " " + str(len(angles)) + " " + str(len(angles)) + " " + str(len(items)))
 
    # Shut down the lidar connection
    lidar.stop()
    lidar.disconnect()
