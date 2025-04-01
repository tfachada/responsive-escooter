# LIBRARIES

## Interfaces
import RPi.GPIO as GPIO
import VL53L0X
from pigpio_dht import DHT11
from mq import MQ
from mpu6050 import *
import serial

## Thread management
import threading
import time

## CSV file management
from datetime import datetime
import csv

## HDTN execution
import os
import subprocess

from math import *


# SETUP AND GLOBAL VARIABLES



## Ignore "channel already in use" warning
GPIO.setwarnings(False)
 
## GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

'''
## Set GPIO Pins
TRIG_PIN_LEFT = 12
ECHO_PIN_LEFT = 6

TRIG_PIN_MIDDLE = 13
ECHO_PIN_MIDDLE = 26

TRIG_PIN_RIGHT = 25
ECHO_PIN_RIGHT = 24
'''

DHT_PIN = 5
XSHUT_LEFT_PIN = 17
XSHUT_MIDDLE_PIN = 27
XSHUT_RIGHT_PIN = 22

'''
## Set GPIO direction (IN / OUT)
GPIO.setup(TRIG_PIN_LEFT, GPIO.OUT)
GPIO.setup(ECHO_PIN_LEFT, GPIO.IN)
GPIO.setup(TRIG_PIN_MIDDLE, GPIO.OUT)
GPIO.setup(ECHO_PIN_MIDDLE, GPIO.IN)
GPIO.setup(TRIG_PIN_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_PIN_RIGHT, GPIO.IN)
'''
GPIO.setup(XSHUT_LEFT_PIN, GPIO.OUT)
GPIO.setup(XSHUT_MIDDLE_PIN, GPIO.OUT)
GPIO.setup(XSHUT_RIGHT_PIN, GPIO.OUT)

# Set all shutdown pins low to turn off each VL53L0X
GPIO.output(XSHUT_LEFT_PIN, GPIO.LOW)
GPIO.output(XSHUT_MIDDLE_PIN, GPIO.LOW)
GPIO.output(XSHUT_RIGHT_PIN, GPIO.LOW)

# Keep all low for 500 ms or so to make sure they reset
time.sleep(0.50)

# Create one object per VL53L0X passing the address to give to
# each.
tof_left = VL53L0X.VL53L0X(address=0x29)
tof_middle = VL53L0X.VL53L0X(address=0x2B)
tof_right = VL53L0X.VL53L0X(address=0x2D)

# Set shutdown pin high for the first VL53L0X then
# call to start ranging
GPIO.output(XSHUT_LEFT_PIN, GPIO.HIGH)
time.sleep(0.50)
tof_left.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

# Set shutdown pin high for the second VL53L0X then
# call to start ranging
GPIO.output(XSHUT_MIDDLE_PIN, GPIO.HIGH)
time.sleep(0.50)
tof_middle.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

# Set shutdown pin high for the third VL53L0X then
# call to start ranging
GPIO.output(XSHUT_RIGHT_PIN, GPIO.HIGH)
time.sleep(0.50)
tof_right.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)


## Init Temperature+Humidity sensor
DHT_SENSOR = DHT11(DHT_PIN)

## State variable for threads
running = True

## Mutex for echo signal to be received
echo_mutex = threading.Lock()

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
        "Smoke" : "",
        "LPG" : "",
        "CO" : ""
}

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


# FUNCTIONS
'''
def measure_distance(trig_pin, echo_pin, location):
    # initial sleep (can result in misreads if not done)
    time.sleep(1)

    while(running):
        with echo_mutex:
            # Send a 10us pulse to trigger pin to start measurement
            GPIO.output(trig_pin, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(trig_pin, GPIO.LOW)

            # Wait for echo pin to go high and start timing
            while GPIO.input(echo_pin) == GPIO.LOW:
                start_time = time.time()

            # Wait for echo pin to go low and stop timing
            while GPIO.input(echo_pin) == GPIO.HIGH:
                end_time = time.time()

        # Calculate distance using speed of sound and time taken
        duration = end_time - start_time
        distance = duration * 17150

        print("Distance (%s) = %.1f cm" % (location, distance))
        data[f"Distance ({location})"] = distance

        time.sleep(1)
'''
def measure_distance(location):
    
    if(location == "left"):
        tof = tof_left
    elif(location == "middle"):
        tof = tof_middle
    else:
        tof = tof_right
    
    while(True):
        try:
            distance = tof.get_distance()
            if (distance > 0):
                print ("Distance (%s): %d mm" % (location, distance))

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
        time.sleep(1)

def measure_air_quality():
    mq = MQ()
    while(running):
        perc = mq.MQPercentage()
        print("Smoke: %g ppm, LPG: %g ppm, CO: %g ppm" % (perc["SMOKE"], perc["GAS_LPG"], perc["CO"]))
        data["Smoke"] = perc["SMOKE"]
        data["LPG"] = perc["GAS_LPG"]
        data["CO"] = perc["CO"]
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

def verify_internet_connection():
    try:
        subprocess.check_output(["ping", "-c", "1", "192.168.1.100"])
        return True
    except subprocess.CalledProcessError:
        return False

def dtn_send():
    
    while running:
        
        time.sleep(5)
        
        # Check internet connection with receiving node
        if verify_internet_connection():
            
            dtn_transfer_start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            print(f"Internet connection available. Initiating DTN transfer at {dtn_transfer_start_time}.")
            
            print(f"Defined source_root: {os.getenv('HDTN_SOURCE_ROOT')}")
            subprocess.call(["cp", csv_file, "/home/pi/HDTN/tests/test_scripts_linux/LCRD_File_Transfer_Test/sender/flightdata"])

            print("Sending file over DTN")
            os.chdir("/home/pi/HDTN/tests/test_scripts_linux/LCRD_File_Transfer_Test/sender")
            dispatch = subprocess.run("./start_ltp_send_bpv6.sh", shell=True)
            
            print("DTN running...")

            if dispatch.returncode == 0:
                print("File transfer successful!")
            else:
                print(f"DTN transfer failed with return code {dispatch.returncode}")
        else:
            print("No internet connection available.")

if __name__ == '__main__':

    # Create a CSV file and write the header
    write_to_file(data.keys())

    # Make the threads run
    try:
        #left_distance_thread = threading.Thread(target=measure_distance, args=(TRIG_PIN_LEFT, ECHO_PIN_LEFT, "left",))
        #middle_distance_thread = threading.Thread(target=measure_distance, args=(TRIG_PIN_MIDDLE, ECHO_PIN_MIDDLE, "middle",))
        #right_distance_thread = threading.Thread(target=measure_distance, args=(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, "right",))
        #left_distance_thread = threading.Thread(target=measure_distance, args=("left",))
        #middle_distance_thread = threading.Thread(target=measure_distance, args=("middle",))
        #right_distance_thread = threading.Thread(target=measure_distance, args=("right",))
        #dht_thread = threading.Thread(target=measure_temperature_humidity)
        air_thread = threading.Thread(target=measure_air_quality)
        #mpu_thread = threading.Thread(target=measure_position_data)
        #gps_thread = threading.Thread(target=get_coordinates)
        #csv_thread = threading.Thread(target=write_latest_values)
        #dtn_thread = threading.Thread(target=dtn_send)
        #fall_thread = threading.Thread(target=fall_detection)
        
        #left_distance_thread.start()
        #middle_distance_thread.start()
        #right_distance_thread.start()
        #dht_thread.start()
        air_thread.start()
        #mpu_thread.start()
        #gps_thread.start()
        #csv_thread.start()
        #dtn_thread.start()
        #fall_thread.start()
        
        #left_distance_thread.join()
        #middle_distance_thread.join()
        #right_distance_thread.join()
        #dht_thread.join()
        air_thread.join()
        #mpu_thread.join()
        #gps_thread.join()
        #csv_thread.join()
        #dtn_thread.join()
        #fall_thread.join()
 
    # CTRL + C -> Wait for threads to finish and leave
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user. Waiting for threads to finish...")
        running = False
        time.sleep(5)
        GPIO.cleanup()
        tof_left.stop_ranging()
        tof_middle.stop_ranging()
        tof_right.stop_ranging()
