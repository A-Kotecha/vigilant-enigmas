#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import numpy as np

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def pos2meters(lon, lat, h):
    R_eq = 6378.137e3
    f = 1/298.257223563
    e2 = f*(2-f)
    CosLat = np.cos(np.radians(lat))
    SinLat = np.sin(np.radians(lat))
    N = R_eq/np.sqrt(1-e2*SinLat**2)
    r = np.zeros(3)
    r[0] = (N + h) * SinLat * np.cos(np.radians(lon))
    r[1] = (N + h) * CosLat * np.sin(np.radians(lon))
    r[2] = h
    return r

def meters2pos(x):
    r = np.array([0.00001*x[0],0.00001*x[1],x[2]])
    return r

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(10)


print("Set default/target airspeed to 4")
vehicle.airspeed = 4
eps = 0.2

def go_to(waypoints):
    wp = []
    for k in waypoints:
        wp.append(meters2pos(k))
    wp = np.array(wp)
    for i in wp:
        while True:
            point = LocationGlobalRelative(i[0],i[1],i[2])
            # Break and return from function just below target altitude.
            vehicle.simple_goto(point)
            x, y, z = pos2meters(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat,
                                 vehicle.location.global_relative_frame.alt)
            xp, yp, zp = pos2meters(point.lon, point.lat, point.alt)
            print(" Position: " + "x: " + str(x) + " y: " + str(y) + " z: " + str(z))
            if (np.sqrt((x - xp) ** 2 + (y - yp) ** 2 + (z - zp) ** 2)) <= eps:
                print("Reached target point !")
                break
            time.sleep(1)

waypoints=[[10,5,15],[10,-5,15],[-10,-5,10],[-10,5,10]]
go_to(waypoints)


'''
while True:
    point1 = LocationGlobalRelative(0.0001, 0.00005, 15)
    # Break and return from function just below target altitude.
    vehicle.simple_goto(point1)
    x, y, z = pos2meters(vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.alt)
    xp1, yp1, zp1 = pos2meters(point1.lon, point1.lat, point1.alt)
    print(" Position: " + "x: "+ str(x)+ " y: "+ str(y) + " z: "+ str(z))
    #if (np.sqrt((vehicle.location.global_relative_frame.lat - point1.lat)**2 + (vehicle.location.global_relative_frame.lon - point1.lon)**2 + 0.001*(vehicle.location.global_relative_frame.alt - point1.alt)**2)) <= eps:
    if (np.sqrt((x - xp1) ** 2 + (y - yp1) ** 2 + (z - zp1) ** 2)) <= eps:
        print("Reached target point !")
        break
    time.sleep(1)

while True:
    point2 = LocationGlobalRelative(0.0001, -0.00005, 15)
    # Break and return from function just below target altitude.
    vehicle.simple_goto(point2)
    x, y, z = pos2meters(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.alt)
    xp2, yp2, zp2 = pos2meters(point2.lon, point2.lat, point2.alt)
    print(" Position: " + "x: "+ str(x)+ " y: "+ str(y) + " z: "+ str(z))
    #if abs(vehicle.location.global_relative_frame.lat) >= abs(point2.lat) * 0.95 and abs(vehicle.location.global_relative_frame.lon) >= abs(point2.lon) * 0.95  :
    #if (np.sqrt((vehicle.location.global_relative_frame.lat - point2.lat)**2 + (vehicle.location.global_relative_frame.lon - point2.lon)**2 + 0.001*(vehicle.location.global_relative_frame.alt - point2.alt)**2)) <= eps:
    if (np.sqrt((x - xp2) ** 2 + (y - yp2) ** 2 + (z - zp2) ** 2)) <= eps:
        print("Reached target point !")
        break
    time.sleep(1)

while True:
    point3 = LocationGlobalRelative(-0.0001, -0.00005, 10)
    # Break and return from function just below target altitude.
    vehicle.simple_goto(point3)
    x, y, z = pos2meters(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.alt)
    xp3, yp3, zp3 = pos2meters(point3.lon, point3.lat, point3.alt)
    print(" Position: " + "x: "+ str(x)+ " y: "+ str(y) + " z: "+ str(z))
    #if abs(vehicle.location.global_relative_frame.lat) >= point3.lat * 0.95 and abs(vehicle.location.global_relative_frame.lon) >= point3.lon * 0.95  :
    #if (np.sqrt((vehicle.location.global_relative_frame.lat - point3.lat)**2 + (vehicle.location.global_relative_frame.lon - point3.lon)**2 + 0.001*(vehicle.location.global_relative_frame.alt - point3.alt)**2)) <= eps:
    if (np.sqrt((x - xp3) ** 2 + (y - yp3) ** 2 + (z - zp3) ** 2)) <= eps:
        print("Reached target point !")
        break
    time.sleep(1)

while True:
    point4 = LocationGlobalRelative(-0.0001, 0.00005, 10)
    # Break and return from function just below target altitude.
    vehicle.simple_goto(point4)
    x, y, z = pos2meters(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.alt)
    xp4, yp4, zp4 = pos2meters(point4.lon, point4.lat, point4.alt)
    print(" Position: " + "x: "+ str(x)+ " y: "+ str(y) + " z: "+ str(z))
    #if abs(vehicle.location.global_relative_frame.lat) >= point4.lat * 0.95 and abs(vehicle.location.global_relative_frame.lon) >= point4.lon * 0.95  :
    #if np.sqrt((vehicle.location.global_relative_frame.lat - point4.lat)**2 + (vehicle.location.global_relative_frame.lon - point4.lon)**2 + 0.001*(vehicle.location.global_relative_frame.alt - point4.alt)**2) <= eps:
    if (np.sqrt((x - xp4) ** 2 + (y - yp4) ** 2 + (z - zp4) ** 2)) <= eps:
        print("Reached target point !")
        break
    time.sleep(1)
'''
# sleep so we can see the change in map
time.sleep(6)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
