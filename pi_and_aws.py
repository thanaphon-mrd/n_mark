# final_new.py
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from dronekit_sitl import SITL
import time
import mysql.connector
import io
import picamera
import boto3
import cv2
import numpy as np
import math
import requests
import json

# Connect to database
db_config = {
    'user': 'root2',
    'password': '7f449104603fc6b60619',
    'host': '203.154.83.233',
    'database': 'gps_data'
}
cnx = mysql.connector.connect(**db_config)
cursor = cnx.cursor()

# Connect to pixhawk
connection_string = "/dev/ttyACM0"  # Use this for USB connectionn | show drivercommand : ls /dev
print("conn pixhawk suc")
baud_rate = 57600
print("Connecting to the vehicle on: %s" % connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True, heartbeat_timeout=120)
print("Vehicle conn Suc")

# Load the fire detection cascade
try:
    fire_cascade = cv2.CascadeClassifier('fire_detection.xml')
except Exception as e:
    print("Error loading cascade file:", e)
    exit()

def get_gps_data(vehicle):
    print("GPS: %s" % vehicle.location.global_relative_frame)
    print("Satellites visible: %s" % vehicle.gps_0.satellites_visible)
    print("GPS Fix: %s" % vehicle.gps_0.fix_type)

def save_image_to_db(cnx, cursor, image_bytes, vehicle):
    print("Saving image to database")

    query = '''
        INSERT INTO gps_records (latitude, longitude, altitude, satellites_visible, gps_fix, image_data)
        VALUES (%s, %s, %s, %s, %s, %s)
    '''
    location = vehicle.location.global_relative_frame
    satellites_visible = vehicle.gps_0.satellites_visible
    gps_fix = vehicle.gps_0.fix_type
    data = (location.lat, location.lon, location.alt, satellites_visible, gps_fix, image_bytes)
    cursor.execute(query, data)
    cnx.commit()

    print("Image saved to database")

def detect_fire(image_bytes):
    rekognition_client = boto3.client('rekognition')

    response = rekognition_client.detect_labels(
        Image={'Bytes': image_bytes},
        MaxLabels=10
    )

    for label in response['Labels']:
        if label['Name'].lower() == 'fire':
            return True

    return False

def takeoff(vehicle):
    # Arm the vehicle
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print('Waiting for arming...')
        time.sleep(1)
    print('Vehicle armed')

    # Take off to 5 meters altitude
    target_altitude = 5
    print('Taking off to %d meters' % target_altitude)
    vehicle.simple_takeoff(5)

    while True:
        altitude = vehicle.location.global_relative_frame.alt
        if altitude >= target_altitude * 0.95:
            print('Reached target altitude')
            break
        time.sleep(1)

def Gotowaypoint(cursor,cnx):
    # Define the SQL query to fetch the waypoints from the database
    sql = "SELECT id, latitude, longitude, altitude, unixtimestamp FROM waypoints ORDER BY id"

    # Execute the query and fetch the results
    cursor.execute(sql)
    waypoints = cursor.fetchall()
    print(waypoints)

    current = int(time.time())
    current = current - 30
    print(current)

    if len(waypoints) == 0:
        print("stand by")

    else:
        if waypoints[0][4] < current :
            takeoff(vehicle)
            print("takeoff suc")

            # Convert the fetched waypoints to LocationGlobalRelative objects
            waypoints_rel = [LocationGlobalRelative(wp[1], wp[2], wp[3]) for wp in waypoints]

            # Navigate to each waypoint in sequence
            for waypoint in waypoints_rel:
                print('Going to waypoint: %s' % waypoint)
                vehicle.simple_goto(waypoint)
                while check_distance_waypoint(vehicle, waypoint, cnx):
                    #
                    if detect_fire_by_raspi(stream,fire_detected,alert_timer,fire_cascade):
                        print("fire detected and save.")
                    else:
                        print("No fire detected.")

            vehicle.mode = VehicleMode('LAND')
            while not vehicle.mode.name == 'LAND':
                print('Waiting for mode change...')
                time.sleep(1)
            print('Vehicle landing')

def detect_fire_by_raspi(stream,fire_detected,alert_timer,fire_cascade):
    # Capture video stream from the camera and process it
    camera.capture(stream, format='jpeg', use_video_port=True)
    # Convert the bytes buffer to a numpy array
    data = np.frombuffer(stream.getvalue(), dtype=np.uint8)

    if data.size == 0:
        print("Error: Empty buffer received")
        return False

    # Decode the numpy array into a color image
    frame = cv2.imdecode(data, cv2.IMREAD_COLOR)

    # Convert the color image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect fire in the grayscale image
    fire = fire_cascade.detectMultiScale(gray, 1.3, 5)

    # Draw a rectangle around the detected fire
    for (x,y,w,h) in fire:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
        fire_detected = True

    # If a fire is detected, display an alert message for 5 seconds
    if fire_detected:
        if alert_timer == 0:
            print("Fire detected in pi!")
            # Encode image bytes as base64 string
            _, img_bytes = cv2.imencode('.png', frame)

            # Call detect_fire function and pass image bytes in PNG format
            if detect_fire(img_bytes.tobytes()):
                print("Fire detected by Amazon Rekognition!")
                save_image_to_db(cnx, cursor, img_bytes.tobytes(), vehicle)
                print("Save image.")
                alert_timer = 10
                return True
            else:
                print("Fire detected in pi but not found by Amazon Rekognition!")
            alert_timer = 10
        else:
            alert_timer -= 1
    else:
        alert_timer = 0
    return False


# Calculate the distance between two points in 3D space
def distance_between_points(lat1, lon1, alt1, lat2, lon2, alt2):
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat / 2) * math.sin(d_lat / 2) +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(d_lon / 2) * math.sin(d_lon / 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = 6371 * c * 1000  # Distance in meters
    d_alt = alt2 - alt1  # Altitude difference in meters
    return math.sqrt(d * d + d_alt * d_alt)

# Delete a waypoint from the database
def delete_waypoint(cnx, id):
    with cnx.cursor() as cursor:
        sql = "DELETE FROM `waypoints` WHERE `id` = %s"
        cursor.execute(sql, (id,))
        cnx.commit()

def check_distance_waypoint(vehicle, waypoint, cnx):
    current_location = vehicle.location.global_relative_frame
    lat1, lon1, alt1 = extract_location_info(current_location)
    lat2, lon2, alt2 = extract_location_info(waypoint)

    distance = distance_between_points(lat1, lon1, alt1, lat2, lon2, alt2)
    print(distance)

    sql = "SELECT id, latitude, longitude, altitude FROM waypoints ORDER BY id"

    # Execute the query and fetch the results
    cursor.execute(sql)
    waypoints = cursor.fetchall()

    threshold = 7  # Set the threshold distance in meters
    result = [item[0] for item in waypoints]
    print("waypoints form checkdistan", result[0])
    if distance < threshold:
        print(f"Waypoint {result[0]} reached. Deleting from the database.")
        delete_waypoint(cnx, result[0])
        
        for i in range(10):
            if detect_fire_by_raspi(stream,fire_detected,alert_timer,fire_cascade):
                print("fire detected and save.")
            else:
                timestamp = time.time()
                camera.capture('/home/pi/Desktop/Test/image' + str(timestamp) + '.jpg')
                print("No fire detected." , i)             
            time.sleep(1)
            i = i + 1
        return False
    return True

def extract_location_info(current_location):
    current_location = str(current_location)
    lat_start = current_location.find("lat=") + 4
    lat_end = current_location.find(",", lat_start)
    lon_start = current_location.find("lon=") + 4
    lon_end = current_location.find(",", lon_start)
    alt_start = current_location.find("alt=") + 4
    alt_end = len(current_location)

    latitude = float(current_location[lat_start:lat_end])
    longitude = float(current_location[lon_start:lon_end])
    altitude = float(current_location[alt_start:alt_end])
    return latitude, longitude, altitude

#......................................................

print("start camera")
with picamera.PiCamera() as camera:
    camera.resolution = (468, 360)
    camera.framerate = 24

    try:
        while True:
            print("Start....Function")
            #get_gps_data(vehicle)
            stream = io.BytesIO()

            # Set up a flag to indicate whether a fire has been detected
            fire_detected = False

            # Set up a timer to prevent repeated alerts for the same fire
            alert_timer = 0

            # waypoints from the database
            Gotowaypoint(cursor,cnx)

            time.sleep(2)

            print("Start....Function....Again")

    except KeyboardInterrupt:
        print("close all and break")
        vehicle.close()
        camera.close()
        stream.seek(0)
        stream.truncate()
        pass

# Close the vehicle
vehicle.close()
print('Connection closed')
