from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Vehicle
import time
import math
from pymavlink import mavutil
import asyncio
#Set up option parsing to get connection string
import argparse
import RPi.GPIO as GPIO
import pyrebase

GPIO.setmode(GPIO.BCM)
GPIO.setup(19, GPIO.OUT)
GPIO.output(19, GPIO.HIGH)
print('mati')
firebaseConfig = {
  "apiKey": "AIzaSyCl9Tl-g54jVLAQ3FkJesQHcsErfrtgoTA",
  "authDomain": "drone-71fff.firebaseapp.com",
  "databaseURL": "https://drone-71fff-default-rtdb.asia-southeast1.firebasedatabase.app",
  "projectId": "drone-71fff",
  "storageBucket": "drone-71fff.appspot.com",
  "messagingSenderId": "1079210623469",
  "appId": "1:1079210623469:web:ab8deb0178bcfe1964e7e4"
    }
    # Deklarasi database firebase
    
firebase = pyrebase.initialize_app(firebaseConfig)
db = firebase.database()

loop = asyncio.get_event_loop()

parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', default='127.0.0.1:14550',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

async def data_database():
    altitude_input = vehicle.location.global_relative_frame.alt
    latitude_input = vehicle.location.global_relative_frame.lat
    longitude_input = vehicle.location.global_relative_frame.lon
    attitude_input = vehicle.attitude
    attitude_pitch_input = vehicle.attitude.pitch
    attitude_yaw_input = vehicle.attitude.yaw
    attitude_roll_input = vehicle.attitude.roll
    groundspeed_input = vehicle.groundspeed
    velocity_input = vehicle.velocity
    battery_voltage_input = vehicle.battery.voltage
    battery_current_input = vehicle.battery.current
    battery_level_input = vehicle.battery.level
    system_status_input = str(vehicle.system_status)
    gps_fix_input = vehicle.gps_0.fix_type
    gps_numsat_input = vehicle.gps_0.satellites_visible
    heading_input = vehicle.heading
    airspeed_input = vehicle.airspeed

    # print("Altitude: ", altitude_input)
    # print("Latitude: ", latitude_input)
    # print("Longitude: ", longitude_input)
    # print("Attitude: ", attitude_input)
    # print("Attitude Pitch: ", attitude_pitch_input)
    # print("Attitude Yaw: ", attitude_yaw_input)
    # print("Attitude Roll: ", attitude_roll_input)
    # print("GroundSpeed: ", groundspeed_input)
    # print("Velocity: ", velocity_input)
    # print("Battery Voltage: ", battery_voltage_input)
    # print("Battery Current: ", battery_current_input)
    # print("Battery Level: ", battery_level_input)
    # print("System Status: ", system_status_input)
    # print("GPS FIX: ", gps_fix_input)
    # print("GPS Satelite: ", gps_numsat_input)
    # print("Heading: ", heading_input)
    # print("Air Speed: ", airspeed_input)

#     print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
#     print ("Airspeed: %s" %vehicle.velocity)
#     print ("groundspeed: %s" %vehicle.groundspeed)
#     print ("attitude: %s" %vehicle.attitude)
#     print ("wind: %s" %vehicle.wind)
#     print ("battery: %s" %vehicle.battery)
#     print ("system status: %s" %vehicle.system_status)
#     print ("battery voltage: %s" %vehicle.voltage)
#     print(" Altitude: ", vehicle.location.global_relative_frame.alt)
#    # print ("location: %s" %vehicle.original_location)
#     #print ("location: %s" %vehicle.)
#     print ("location1: %s" %vehicle.location.global_frame)
#     print ("location2: %s" %vehicle.location.global_relative_frame)
#     print ("location3: %s" %vehicle.commands.next.x)
#     print ("location waypoints: %s" %LocationGlobalRelative(lat,lon))
#     print ("GPS: %s" %vehicle.gps_0)
#     print ("heading on compass(degree): %s" %vehicle.heading)
    altitude_input = vehicle.location.global_relative_frame.alt
    latitude_input = vehicle.location.global_relative_frame.lat
    longitude_input = vehicle.location.global_relative_frame.lon
    groundspeed_input = vehicle.groundspeed
    a = time.asctime( time.localtime(time.time()) )
    waktu_altitude = (str(altitude_input) + " " + a)
    waktu_latitude = (str(latitude_input) + " " + a)
    waktu_longitude = (str(longitude_input) + " " + a)
    waktu_groundspeed = (str(groundspeed_input) + " " + a)
    # await asyncio.sleep(0.01)
    return altitude_input, latitude_input, longitude_input, attitude_input, attitude_pitch_input, attitude_yaw_input, attitude_roll_input, groundspeed_input, velocity_input, battery_voltage_input, battery_current_input, battery_level_input, system_status_input, gps_fix_input, gps_numsat_input, heading_input, airspeed_input, waktu_altitude, waktu_latitude, waktu_longitude, waktu_groundspeed
        

async def dic_database():
    ketinggian = {"Altitude": data_database()[0]}
    longitude = {"Longitude": data_database()[2]}
    groundspeed = {"Ground_Speed": data_database()[7]}
    latitude = {"Latitude": data_database()[1]}
    time_altitude = {"Altitude(Waktu)": data_database()[17]}
    time_latitude = {"Latitude(Waktu)": data_database()[18]}
    time_longitude = {"Longitude(Waktu)": data_database()[19]}
    time_groundspeed = {"Groundspeed(Waktu)": data_database()[20]}
    return ketinggian, longitude, groundspeed, latitude, time_altitude, time_latitude, time_longitude, time_groundspeed

async def push_database():
    db.child("Altitude").push(dic_database()[0])
    db.child("Altitude (Tampil)").set(dic_database()[0])
    db.child("Longitude").push(dic_database()[1])
    db.child("Longitude (Tampil)").set(dic_database()[1])
    db.child("Ground Speed").push(dic_database()[2])
    db.child("Ground Speed (Tampil)").set(dic_database()[2])
    db.child("Latitude").push(dic_database()[3])
    db.child("Latitude (Tampil)").set(dic_database()[3])
    db.child("Altitude(Waktu)").push(dic_database()[4])
    db.child("Latitude(Waktu)").push(dic_database()[5])
    db.child("Longitude(Waktu)").push(dic_database()[6])
    db.child("Groundspeed(Waktu)").push(dic_database()[7])
    time.sleep(1)
    await asyncio.sleep(0.01)

async def main():
#New Line Added
    while True:
        f1 = loop.create_task(data_database())
        f2 = loop.create_task(dic_database())
        f3 = loop.create_task(push_database())
        await asyncio.wait([f1, f2, f3])


def variabel():
    altitude_input = vehicle.location.global_relative_frame.alt
    latitude_input = vehicle.location.global_relative_frame.lat
    longitude_input = vehicle.location.global_relative_frame.lon
    attitude_input = vehicle.attitude
    attitude_pitch_input = vehicle.attitude.pitch
    attitude_yaw_input = vehicle.attitude.yaw
    attitude_roll_input = vehicle.attitude.roll
    groundspeed_input = vehicle.groundspeed
    velocity_input = vehicle.velocity
    battery_voltage_input = vehicle.battery.voltage
    battery_current_input = vehicle.battery.current
    battery_level_input = vehicle.battery.level
    system_status_input = str(vehicle.system_status)
    gps_fix_input = vehicle.gps_0.fix_type
    gps_numsat_input = vehicle.gps_0.satellites_visible
    heading_input = vehicle.heading
    airspeed_input = vehicle.airspeed

    print("Altitude: ", altitude_input)
    print("Latitude: " , latitude_input)
    print("Longitude: ",  longitude_input)
    print("Attitude: ",  attitude_input)
    print("Attitude Pitch: ",  attitude_pitch_input)
    print("Attitude Yaw: ",  attitude_yaw_input)
    print("Attitude Roll: ",  attitude_roll_input)
    print("GroundSpeed: ",  groundspeed_input)
    print("Velocity: ",  velocity_input)
    print("Battery Voltage: ",  battery_voltage_input)
    print("Battery Current: ",  battery_current_input)
    print("Battery Level: ",  battery_level_input)
    print("System Status: ",  system_status_input)
    print("GPS FIX: ",  gps_fix_input)
    print("GPS Satelite: ",  gps_numsat_input)
    print("Heading: ",  heading_input)
    print("Air Speed: ",  airspeed_input)

    print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
    print ("Airspeed: %s" %vehicle.velocity)
    print ("groundspeed: %s" %vehicle.groundspeed)
    print ("attitude: %s" %vehicle.attitude)
    print ("wind: %s" %vehicle.wind)
    print ("battery: %s" %vehicle.battery)
    print ("system status: %s" %vehicle.system_status)
    #print ("battery voltage: %s" %vehicle.voltage)
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    #print ("location: %s" %vehicle.original_location)
    #print ("location: %s" %vehicle.)
    print ("location1: %s" %vehicle.location.global_frame)
    print ("location2: %s" %vehicle.location.global_relative_frame)
    #print ("location3: %s" %vehicle.commands.next.x)
    #print ("location waypoints: %s" %LocationGlobalRelative(lat))
    print ("GPS: %s" %vehicle.gps_0)
    print ("heading on compass(degree): %s" %vehicle.heading)
    altitude_input = vehicle.location.global_relative_frame.alt
    latitude_input = vehicle.location.global_relative_frame.lat
    longitude_input = vehicle.location.global_relative_frame.lon
    groundspeed_input = vehicle.groundspeed


    print("data altitude : " , altitude_input)
    print("data latitude : " , latitude_input)
    print("data longitude : " , longitude_input)
    print("data groundspeed : " , groundspeed_input)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)
        print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        # push_database()  
        loop = asyncio.get_event_loop()
        loop.run_until_complete(main())
        loop.close()  
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

        
print('Create a new mission (for current location)')
download_mission()


# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(1)

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")


# Monitor mission. 

while True:
    nextwaypoint=vehicle.commands.next
     # Variabel
    # variabel()
    # push_database()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()
    print(dic_database()[0])
    if nextwaypoint== 2 or nextwaypoint== 5 or nextwaypoint== 6:
         GPIO.output(19, GPIO.LOW)
         print("GPIO LOW (NYALA)")
         print(nextwaypoint)
    elif nextwaypoint== 3 or nextwaypoint== 4:
         GPIO.output(19, GPIO.HIGH)
         print("GPIO HIGH (MATI")
         print(nextwaypoint)
    elif nextwaypoint== 9 : #Dummy waypoint - as soon as we reach waypoint 11 this is true and we exit.
         print("Exit 'standard' mission when start heading to final waypoint (11)")
         print(nextwaypoint)
         break
    time.sleep(1)
vehicle.mode == VehicleMode("RTL")

print('Return to launch')

while True:
    if vehicle.location.global_relative_frame.alt>= 0:
        # push_database()
        loop = asyncio.get_event_loop()
        loop.run_until_complete(main())
        loop.close()
    else:
        break
#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
