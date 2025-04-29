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

## Values of constants (names in CAPS)
from lib.constants import *


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

## Button pins
GPIO.setup(DANGER_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(WET_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(POLLUTION_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(DRIVING_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

## LED pins
GPIO.setup(DANGER_LED, GPIO.OUT)
GPIO.setup(WET_LED, GPIO.OUT)
GPIO.setup(POLLUTION_LED, GPIO.OUT)
GPIO.setup(DRIVING_LED, GPIO.OUT)

## Vibration motor pin
GPIO.setup(VIBRATION_PIN, GPIO.OUT)

## State variable for threads
running = True

## Mutex for the CSV values to match their timestamp
timestamp_mutex = threading.Lock()

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
        "AQI" : ""
}

## Distance variables
distance_left = 100
distance_middle = 100
distance_right = 100

## Accelerations
#global Ax
#global Ay
#global Az
#Ax = 0
#Ay = 0
#Az = 0

## Scooter inclination (y axis of sensor)
#global Gy
#Gy = 0


# STATUS INDICATORS
road_danger = False
wet_pavement = False
high_humidity = False
pollution = False
dangerous_driving = False
forbidden_zone = False


# FUNCTIONS

def measure_distance(sensor, location, distance):
    while(running):
        try:
            distance = sensor.get_distance()
            print ("Distance (%s): %d mm" % (location, distance))

            if distance < 500:
                road_danger = True
            else:
                road_danger = False

            time.sleep(1)
        except KeyboardInterrupt:
            break

def measure_temperature_humidity():
    while(running):
        try:
            result = DHT_SENSOR.read()
            temperature = result["temp_c"]
            humidity = result["humidity"]
            print("Temperature: %.1f ºC" % temperature)
            print("Humidity: %.1f" % humidity + "%")
            data["Temperature"] = temperature
            data["Humidity"] = humidity
        except TimeoutError:
            pass

        if True:
            high_humidity = True
        else:
            high_humidity = False

        time.sleep(1)

def measure_air_quality():
    mq = MQ()
    while(running):
        aqi = mq.MQPercentage()["SMOKE"]
        print("Air Quality Index: %.1f" % aqi)
        data["AQI"] = aqi

        if True:
            pollution = True
        else:
            pollution = False

        time.sleep(1)

def measure_position_data():
    
    MPU_Init()

    while running:
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
        if Ax < 1 or abs(Az) > 0.5:
            print("FALLING")
        if abs(Gy) > 5:
            print("FALLING!!!")

        # TODO: Cross-check with distances

        print("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
        sleep(0.5)

def get_coordinates():
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

                    # Write data to CSV file
                    #writer.writerow({'Timestamp': timestamp, 'Latitude': latitude, 'Longitude': longitude, 'Speed': speed, 'Altitude': altitude})
                    #csvfile.flush()
                    
                    print(f"Timestamp: {timestamp}, Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}, Speed: {speed}")
                    data["Latitude"] = latitude
                    data["Longitude"] = longitude
                    data["Altitude"] = altitude
                    data["Speed"] = speed
            except IndexError:
                pass

            # TODO: different variable for this one
            if True:
                road_danger = True
            else:
                road_danger = False

            if True:
                wet_pavement = True
            else:
                wet_pavement = False

            if True:
                forbidden_zone = True
            else:
                forbidden_zone = False

            # Wait for a few seconds before sending the next command
            time.sleep(5)

    finally:
        # Disable GNSS module
        ser.write(b'AT+CGNSPWR=0\r\n')
        time.sleep(1)

        # Close serial port
        ser.close()

def write_to_file(row_of_values):
    with open(csv_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(row_of_values)

def write_latest_values():
    while running:
        time.sleep(5)
        with timestamp_mutex:
            data["Date"] = datetime.today().strftime("%Y-%m-%d")
            data["Timestamp"] = datetime.today().strftime("%H:%M:%S")
            write_to_file(data.values())

def check_status_aux(status, led, button, button_pressed):

    if status == True:
        if button_pressed == False:
            GPIO.output(led, GPIO.HIGH)
        if GPIO.input(button) == GPIO.HIGH:
            button_pressed = True
    else:
        GPIO.output(led, GPIO.LOW)
        button_pressed = False


def check_status():

    danger_button_pressed = False
    wet_button_pressed = False
    pollution_button_pressed = False
    driving_button_pressed = False

    if forbidden_zone:
        GPIO.output(VIBRATION_PIN, GPIO.HIGH)
        GPIO.output(DANGER_LED, GPIO.HIGH)
        GPIO.output(WET_LED, GPIO.HIGH)
        GPIO.output(POLLUTION_LED, GPIO.HIGH)
        GPIO.output(DRIVING_LED, GPIO.HIGH)
        time.sleep(.5)
        GPIO.output(VIBRATION_PIN, GPIO.LOW)
        GPIO.output(DANGER_LED, GPIO.LOW)
        GPIO.output(WET_LED, GPIO.LOW)
        GPIO.output(POLLUTION_LED, GPIO.LOW)
        GPIO.output(DRIVING_LED, GPIO.LOW)

    else:
        check_status_aux(road_danger, DANGER_LED, DANGER_BUTTON, danger_button_pressed)
        check_status_aux(high_humidity, WET_LED, WET_BUTTON, wet_button_pressed)
        check_status_aux(wet_pavement, WET_LED, WET_BUTTON, wet_button_pressed)
        check_status_aux(pollution, POLLUTION_LED, POLLUTION_BUTTON, pollution_button_pressed)
        check_status_aux(dangerous_driving, DRIVING_LED, DRIVING_BUTTON, driving_button_pressed)

        if road_danger or high_humidity or wet_pavement or pollution or dangerous_driving:
            GPIO.output(VIBRATION_PIN, GPIO.HIGH)
        else:
            GPIO.output(VIBRATION_PIN, GPIO.LOW)

    time.sleep(1)


if __name__ == '__main__':

    # Create a CSV file and write the header
    write_to_file(data.keys())

    # Make the threads run
    try:
        #left_distance_thread = threading.Thread(target=measure_distance, args=(tof_left, "left", distance_left,))
        #middle_distance_thread = threading.Thread(target=measure_distance, args=(tof_middle, "middle", distance_middle,))
        #right_distance_thread = threading.Thread(target=measure_distance, args=(tof_right, "right", distance_right,))
        #mpu_thread = threading.Thread(target=measure_position_data)
        dht_thread = threading.Thread(target=measure_temperature_humidity)
        #air_thread = threading.Thread(target=measure_air_quality)
        #gps_thread = threading.Thread(target=get_coordinates)
        
        #csv_thread = threading.Thread(target=write_latest_values)
        #sending_thread = threading.Thread(target=multicast_sender, args=(csv_file, MCAST_GROUP_A, MCAST_PORT,), daemon = True)
        #receiving_thread = threading.Thread(target=multicast_receiver, args=(RECV_OUTPUT_DIR, MCAST_GROUP_B, MCAST_PORT,), daemon = True)

        actuation_thread = threading.Thread(target=check_status)
        

        #left_distance_thread.start()
        #middle_distance_thread.start()
        #right_distance_thread.start()
        #mpu_thread.start()
        dht_thread.start()
        #air_thread.start()
        #gps_thread.start()
        
        #csv_thread.start()
        #sending_thread.start()
        #receiving_thread.start()

        actuation_thread.start()
        

        #left_distance_thread.join()
        #middle_distance_thread.join()
        #right_distance_thread.join()
        #mpu_thread.join()
        dht_thread.join()
        #air_thread.join()
        #gps_thread.join()

        #csv_thread.join()
        #sending_thread.join()
        #receiving_thread.join()

        actuation_thread.join()
 
    # CTRL + C -> Wait for threads to finish and leave
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user. Waiting for threads to finish...")
        running = False
        time.sleep(5)
        GPIO.cleanup()
        tof_left.stop_ranging()
        tof_middle.stop_ranging()
        tof_right.stop_ranging()
