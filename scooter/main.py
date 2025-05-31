#!venv/bin/python

# LIBRARIES

## Interfaces
import RPi.GPIO as GPIO
import serial
from pigpio_dht import DHT11

import lib.VL53L0X as VL53L0X
from lib.mq import MQ
from lib.mpu6050 import *

## Thread management
import threading
import time

## CSV file management
from datetime import datetime
import csv

## Communication
from lib.multicast import *

## Intermediate node .json data parsing
import json

## Values of constants (names in CAPS)
from lib.constants import *

## Nominatim mode (set by optional argument)
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--nominatim", action="store_true")
args = parser.parse_args()
if args.nominatim:
    from geopy.geocoders import Nominatim
    from functools import partial


# SETUP

## Ignore "channel already in use" warning
GPIO.setwarnings(False)

## GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

GPIO.setup(XSHUT_LEFT_PIN, GPIO.OUT)
GPIO.setup(XSHUT_MIDDLE_PIN, GPIO.OUT)
GPIO.setup(XSHUT_RIGHT_PIN, GPIO.OUT)

## Set all shutdown pins low to turn off each VL53L0X
GPIO.output(XSHUT_LEFT_PIN, GPIO.LOW)
GPIO.output(XSHUT_MIDDLE_PIN, GPIO.LOW)
GPIO.output(XSHUT_RIGHT_PIN, GPIO.LOW)

## Keep all low for 500 ms or so to make sure they reset
time.sleep(0.50)

## Create one object per VL53L0X passing the address to give to each.
tof_left = VL53L0X.VL53L0X(address=TOF_LEFT_ADDRESS)
tof_middle = VL53L0X.VL53L0X(address=TOF_MIDDLE_ADDRESS)
tof_right = VL53L0X.VL53L0X(address=TOF_RIGHT_ADDRESS)

## Set shutdown pin high for the first VL53L0X then call to start ranging
GPIO.output(XSHUT_LEFT_PIN, GPIO.HIGH)
time.sleep(0.50)
tof_left.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

## Set shutdown pin high for the second VL53L0X then call to start ranging
GPIO.output(XSHUT_MIDDLE_PIN, GPIO.HIGH)
time.sleep(0.50)
tof_middle.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

## Set shutdown pin high for the third VL53L0X then call to start ranging
GPIO.output(XSHUT_RIGHT_PIN, GPIO.HIGH)
time.sleep(0.50)
tof_right.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)


## Init Temperature+Humidity sensor
DHT_SENSOR = DHT11(DHT_PIN)

## Init Air Quality sensor
try:
    mq = MQ()
except ZeroDivisionError:
    pass

## Init Accelerometer
try:
    MPU_Init()
except OSError:
    pass


## Button pins
GPIO.setup(DANGER_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(WEATHER_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(POLLUTION_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(NORIDE_REPORT_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

## LED pins
GPIO.setup(DANGER_LED, GPIO.OUT)
GPIO.setup(WEATHER_LED, GPIO.OUT)
GPIO.setup(POLLUTION_LED, GPIO.OUT)
GPIO.setup(DRIVING_LED, GPIO.OUT)

## Vibration motor pin
GPIO.setup(VIBRATION_PIN, GPIO.OUT)

## State variable for threads
running = True

## Name of the CSV file for this trip
csv_file = datetime.today().strftime("%Y%m%d_%H%M%S") + ".csv"

## Values to be sent
data = {
    "Date" : "",
    "Timestamp" : "",
    "Latitude" : "",
    "Longitude" : "",
    "Altitude" : "",
    "Speed" : "",
    "Distance (left)" : "",
    "Distance (middle)" : "",
    "Distance (right)" : "",
    "Temperature" : "",
    "Humidity" : "",
    "AQI" : "",
    "No-ride report" : "",
    "Danger report" : "",
    "Weather report" : "",
    "Pollution report" : ""
}

## Distance variables
distance_left = 10000
distance_middle = 10000
distance_right = 10000

## Coordinates (used by the geolocation thread)
latitude = -1
longitude = -1

## Timestamp (updated by the GPS module)
timestamp = datetime.today().strftime("%H:%M:%S")


# STATUS INDICATORS

## Sensed and geofenced data

dangerous_driving = False

road_danger_left = False
road_danger_middle = False
road_danger_right = False
road_danger_gps = False

unfavorable_weather_sensors = False
wet_pavement = False

pollution = False

noride_zone = False
noride_zone_nominatim = False

## Button presses
danger_button_pressed = False
weather_button_pressed = False
pollution_button_pressed = False
noride_report_button_pressed = False

## Mutexes
i2c_mutex = threading.Lock()

## Sampling and processing intervals
distance_interval = 1
dht_interval = 1
mpu_interval = 0.5
air_interval = 0
gps_interval = 3
json_interval = 5
nominatim_interval = 0
logging_interval = 5
check_status_interval = 1


# FUNCTIONS

def measure_distance(sensor, location, distance):

    global road_danger_left
    global road_danger_middle
    global road_danger_right

    while(running):
        try:
            with i2c_mutex:
                distance = sensor.get_distance()
            
            print ("Distance (%s): %d mm" % (location, distance))

            if sensor == tof_left:
                if distance < 500:
                    road_danger_left = True
                else:
                    road_danger_left = False

            elif sensor == tof_middle:
                if distance < 500:
                    road_danger_middle = True
                else:
                    road_danger_middle = False

            else:
                if distance < 500:
                    road_danger_right = True
                else:
                    road_danger_right = False

            time.sleep(distance_interval)
        except KeyboardInterrupt:
            break

def measure_temperature_humidity():

    global unfavorable_weather_sensors

    while(running):
        try:
            result = DHT_SENSOR.read()
            temperature = result["temp_c"]
            humidity = result["humidity"]
            print("Temperature: %.1f ºC" % temperature)
            print("Humidity: %.1f" % humidity + "%")
            data["Temperature"] = temperature
            data["Humidity"] = humidity

            if humidity > 60 or temperature > 30:
                unfavorable_weather_sensors = True
            else:
                unfavorable_weather_sensors = False

        except TimeoutError:
            pass

        time.sleep(dht_interval)

def measure_air_quality():

    global pollution

    while(running):
        aqi = mq.MQPercentage()["SMOKE"]
        print("Air Quality Index: %f" % aqi)
        data["AQI"] = aqi

        if aqi > 0.15:
            pollution = True
        else:
            pollution = False
        
        time.sleep(air_interval)

def measure_position_data():

    global dangerous_driving

    while running:
        with i2c_mutex:
            # Read Accelerometer raw value
            acc_x = read_raw_data(ACCEL_XOUT_H)
            acc_y = read_raw_data(ACCEL_YOUT_H)
            acc_z = read_raw_data(ACCEL_ZOUT_H)

            # Read Gyroscope raw value
            gyro_x = read_raw_data(GYRO_XOUT_H)
            gyro_y = read_raw_data(GYRO_YOUT_H)
            gyro_z = read_raw_data(GYRO_ZOUT_H)

        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0

        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0

        # Fall detection
        if abs(Gy) > 5:
            dangerous_driving = True
        else:
            dangerous_driving = False

        print("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
        sleep(mpu_interval)

def get_coordinates():

    global latitude
    global longitude
    global timestamp

    # Open serial port
    ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)

    # Enable GNSS module
    ser.write(b'AT+CGNSPWR=1\r\n')
    time.sleep(1)

    try:
        while running:
            # Request location information
            ser.write(b'AT+CGNSINF\r\n')
            time.sleep(1)

            # Read response from module
            response = ser.read_all().decode('utf-8')
            print(response)

            # Extract data
            gps_data = response.split(',')
            try:
                if gps_data[1] == '1':  # Check if the GPS module has acquired a fix
                    timestamp = gps_data[2]
                    latitude = gps_data[3]
                    longitude = gps_data[4]
                    speed = gps_data[6]
                    altitude = gps_data[5]

                    print(f"Timestamp: {timestamp}, Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}, Speed: {speed}")
                    data["Latitude"] = latitude
                    data["Longitude"] = longitude
                    data["Altitude"] = altitude
                    data["Speed"] = speed
                else:
                    timestamp = datetime.today().strftime("%H:%M:%S")   # Default timestamp value
            except IndexError:
                pass

            # Wait for a few seconds before sending the next command
            time.sleep(gps_interval)

    finally:
        # Disable GNSS module
        ser.write(b'AT+CGNSPWR=0\r\n')
        time.sleep(1)

        # Close serial port
        ser.close()

def check_map_data():

    global road_danger_gps
    global wet_pavement
    global noride_zone

    while(running):

        # Default values
        map_values = {
            "road_danger": 0,
            "wet_pavement": 0,
            "noride_zone": 0
        }

        # Read the JSON file
        with open("server_data.json", "r") as file:
            data = json.load(file)

        # Iterate through the data
        for entry in data:
            if (entry["min_lon"] <= float(longitude) <= entry["max_lon"] and
                entry["min_lat"] <= float(latitude) <= entry["max_lat"]):
                map_values["road_danger"] = entry["road_danger"]
                map_values["wet_pavement"] = entry["wet_pavement"]
                map_values["noride_zone"] = entry["noride_zone"]
                break

        if map_values["road_danger"] == 1:
            road_danger_gps = True
        else:
            road_danger_gps = False

        if map_values["wet_pavement"] == 1:
            wet_pavement = True
        else:
            wet_pavement = False

        if map_values["noride_zone"] == 1:
            noride_zone = True
        else:
            noride_zone = False

        time.sleep(json_interval)

def check_map_data_nominatim():

    global noride_zone_nominatim

    while(running):

        try:
            geolocator = Nominatim(domain="localhost:8080", scheme="http", user_agent="GMMS_scooter")
            reverse = partial(geolocator.reverse, language="en")

            try:
                place_type = reverse("%f, %f" % (latitude, longitude)).raw["class"]
            except AttributeError:
                print("Error. Invalid coordinates.")

            if place_type == "amenity":
                noride_zone_nominatim = True
            else:
                noride_zone_nominatim = False

        except ConnectionRefusedError:
            print("Error. If running with --nominatim, please turn on Nominatim, and wait for its initialization.")
        
        time.sleep(nominatim_interval)

def write_to_file(row_of_values):
    with open(csv_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(row_of_values)

def write_latest_values():
    while running:
        time.sleep(logging_interval)
        data["Date"] = datetime.today().strftime("%Y-%m-%d")
        data["Timestamp"] = timestamp
        write_to_file(data.values())

# Checks if a special status needs to be triggered, and turns off LED if user pressed button to dismiss. Called by check_status.
def check_status_aux(status, led, button, button_pressed):

    if status == True:
        if button_pressed == False:
            GPIO.output(led, GPIO.HIGH)
            print("Special status detected. Warning user...")
        if GPIO.input(button) == GPIO.HIGH:
            return True
    else:
        GPIO.output(led, GPIO.LOW)
        return False


def check_status():

    global danger_button_pressed
    global weather_button_pressed
    global pollution_button_pressed

    while(running):

        # Flash periodically if the user cannot ride there. Cannot be dismissed.
        if noride_zone or noride_zone_nominatim:
            print("No-ride zone!")
            GPIO.output(VIBRATION_PIN, GPIO.HIGH)
            GPIO.output(DANGER_LED, GPIO.HIGH)
            GPIO.output(WEATHER_LED, GPIO.HIGH)
            GPIO.output(POLLUTION_LED, GPIO.HIGH)
            GPIO.output(DRIVING_LED, GPIO.HIGH)
            
            time.sleep(check_status_interval / 2)
            
            GPIO.output(VIBRATION_PIN, GPIO.LOW)
            GPIO.output(DANGER_LED, GPIO.LOW)
            GPIO.output(WEATHER_LED, GPIO.LOW)
            GPIO.output(POLLUTION_LED, GPIO.LOW)
            GPIO.output(DRIVING_LED, GPIO.LOW)

        else:
            # Aglomerate equivalent cases (different sensors, use of GPS...)
            road_danger = road_danger_left or road_danger_middle or road_danger_right or road_danger_gps
            unfavorable_weather = unfavorable_weather_sensors or wet_pavement

            # Check LED/Button status (dangerous driving only has a LED)
            check_status_aux(dangerous_driving, DRIVING_LED, False, False)
            danger_button_pressed = check_status_aux(road_danger, DANGER_LED, DANGER_BUTTON, danger_button_pressed)
            weather_button_pressed = check_status_aux(unfavorable_weather, WEATHER_LED, WEATHER_BUTTON, weather_button_pressed)
            pollution_button_pressed = check_status_aux(pollution, POLLUTION_LED, POLLUTION_BUTTON, pollution_button_pressed)

            # Vibrate for any special status
            if dangerous_driving or road_danger or unfavorable_weather or pollution:
                GPIO.output(VIBRATION_PIN, GPIO.HIGH)
            else:
                GPIO.output(VIBRATION_PIN, GPIO.LOW)

            # Ability to report any non-detected status (except dangerous driving) with a button

            if GPIO.input(NORIDE_REPORT_BUTTON) == GPIO.HIGH:
                data["No-ride report"] = True
            else:
                data["No-ride report"] = False

            if GPIO.input(DANGER_BUTTON) == GPIO.HIGH and not danger_button_pressed:
                data["Danger report"] = True
            else:
                data["Danger report"] = False

            if GPIO.input(WEATHER_BUTTON) == GPIO.HIGH and not weather_button_pressed:
                data["Weather report"] = True
            else:
                data["Weather report"] = False

            if GPIO.input(POLLUTION_BUTTON) == GPIO.HIGH and not pollution_button_pressed:
                data["Pollution report"] = True
            else:
                data["Pollution report"] = False

        time.sleep(check_status_interval)


if __name__ == '__main__':

    # Create a CSV file and write the header
    write_to_file(data.keys())

    # Make the threads run
    try:
        left_distance_thread = threading.Thread(target=measure_distance, args=(tof_left, "left", distance_left,))
        middle_distance_thread = threading.Thread(target=measure_distance, args=(tof_middle, "middle", distance_middle,))
        right_distance_thread = threading.Thread(target=measure_distance, args=(tof_right, "right", distance_right,))
        mpu_thread = threading.Thread(target=measure_position_data)
        dht_thread = threading.Thread(target=measure_temperature_humidity)
        air_thread = threading.Thread(target=measure_air_quality)
        gps_thread = threading.Thread(target=get_coordinates)

        map_json_thread = threading.Thread(target=check_map_data)

        csv_thread = threading.Thread(target=write_latest_values)
        sending_thread = threading.Thread(target=multicast_sender, args=(csv_file, MCAST_GROUP_A, MCAST_PORT,), daemon = True)
        receiving_thread = threading.Thread(target=multicast_receiver, args=(RECV_OUTPUT_DIR, MCAST_GROUP_B, MCAST_PORT,), daemon = True)

        actuation_thread = threading.Thread(target=check_status)


        left_distance_thread.start()
        middle_distance_thread.start()
        right_distance_thread.start()
        mpu_thread.start()
        dht_thread.start()
        air_thread.start()
        gps_thread.start()

        map_json_thread.start()

        csv_thread.start()
        sending_thread.start()
        receiving_thread.start()

        actuation_thread.start()


        left_distance_thread.join()
        middle_distance_thread.join()
        right_distance_thread.join()
        mpu_thread.join()
        dht_thread.join()
        air_thread.join()
        gps_thread.join()

        map_json_thread.join()

        csv_thread.join()
        sending_thread.join()
        receiving_thread.join()

        actuation_thread.join()


        if args.nominatim:
            map_db_thread = threading.Thread(target=check_map_data_nominatim)
            map_db_thread.start()
            map_db_thread.join()

    # CTRL + C -> Wait for threads to finish and leave
    except KeyboardInterrupt:
        print("\nScript stopped by user. Waiting for threads to finish...")
        running = False
        time.sleep(5)
        GPIO.cleanup()
        tof_left.stop_ranging()
        tof_middle.stop_ranging()
        tof_right.stop_ranging()
