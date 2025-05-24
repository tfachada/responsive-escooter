from pigpio_dht import DHT11 # type: ignore

DHT_PIN = 5

# Initialize DHT11 sensor
DHT_SENSOR = DHT11(DHT_PIN)

result = DHT_SENSOR.read()
new_temp = result['temp_c']
new_hum = result['humidity']

print(f"Temperature: {new_temp}")
print(f"Humidity: {new_hum}")

