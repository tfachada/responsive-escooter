# Button pins
DANGER_BUTTON = 6
WET_BUTTON = 13
POLLUTION_BUTTON = 19
DRIVING_BUTTON = 26

# LED pins
DANGER_LED = 12
WET_LED = 16
POLLUTION_LED = 20
DRIVING_LED = 21

# Vibration motor pin
VIBRATION_PIN = 1

# Humidity sensor data
DHT_PIN = 5

# Reset pin for ToF sensors
XSHUT_LEFT_PIN = 17
XSHUT_MIDDLE_PIN = 27
XSHUT_RIGHT_PIN = 22

# I2C addresses of ToF sensors
TOF_LEFT_ADDRESS = 0x29
TOF_MIDDLE_ADDRESS = 0x2B
TOF_RIGHT_ADDRESS = 0x2D

# Communication settings
MCAST_GROUP_A = "239.192.0.1"   # Scooter to Intermediate Node
MCAST_GROUP_B = "239.192.0.2"   # Intermediate Node to Scooter
MCAST_PORT = 5000
RECV_OUTPUT_DIR = "./"
