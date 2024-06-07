import time
import board
import busio
import microcontroller
import wifi
import adafruit_requests
import adafruit_scd4x
from digitalio import DigitalInOut, Direction, Pull
from adafruit_pm25.i2c import PM25_I2C
import adafruit_minimqtt.adafruit_minimqtt as MQTT

try:
    from secrets import secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    raise

def handle_exceptions(func):
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            print("Error:", str(e))
            microcontroller.reset()
    return wrapper

@handle_exceptions
def main():
    print("Connecting to %s" % secrets["ssid"])
    wifi.radio.connect(secrets["ssid"], secrets["password"])
    print("Connected to %s!" % secrets["ssid"])

    # MQTT and sensor setup omitted for brevity. Add your existing setup here.

    i2c = busio.I2C(scl=board.SCL1, sda=board.SDA1, frequency=100000)
    scd4x = adafruit_scd4x.SCD4X(i2c)
    reset_pin = None

    pm25 = PM25_I2C(i2c, reset_pin)
    print("Found PM2.5 sensor, reading data..")
    print("Serial number:", [hex(i) for i in scd4x.serial_number])

    scd4x.start_periodic_measurement()
    print("Waiting for first SCD41 measurement....")

    while True:
        if scd4x.data_ready:
            print("CO2: %d ppm" % scd4x.CO2)
            print("Temperature: %0.1f *C" % scd4x.temperature)
            print("Humidity: %0.1f %%" % scd4x.relative_humidity)
            print()

        aqdata = pm25.read()
        print(aqdata)
        
        # MQTT publishing logic here.

        time.sleep(10)

if __name__ == "__main__":
    main()
