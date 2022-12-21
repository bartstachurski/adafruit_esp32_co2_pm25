import time
import board
import busio
import ipaddress
import ssl
import socketpool
import wifi
import adafruit_requests
import adafruit_scd4x
from digitalio import DigitalInOut, Direction, Pull
from adafruit_pm25.i2c import PM25_I2C
import adafruit_minimqtt.adafruit_minimqtt as MQTT

# pylint: disable=no-name-in-module,wrong-import-order
try:
    from secrets import secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    raise

print("Connecting to %s" % secrets["ssid"])
wifi.radio.connect(secrets["ssid"], secrets["password"])
print("Connected to %s!" % secrets["ssid"])

mqtt_topic = "test/topic"
co2_topic = "air_sensor/co2"
temperature_topic = "air_sensor/temperature"
humidity_topic = "air_sensor/humidity"
pm10_standard_topic = "air_sensor/pm10_standard"
pm25_standard_topic = "air_sensor/pm25_standard"
pm100_standard_topic = "air_sensor/pm100_standard"
pm10_env_topic = "air_sensor/pm10_env"
pm25_env_topic = "air_sensor/pm25_env"
pm100_env_topic = "air_sensor/pm100_env"
particles_03um_topic = "air_sensor/particles_03um"
particles_05um_topic = "air_sensor/particles_05um"
particles_10um_topic = "air_sensor/particles_10um"
particles_25um_topic = "air_sensor/particles_25um"
particles_50um_topic = "air_sensor/particles_50um"
particles_100um_topic = "air_sensor/particles_100um"

### Code ###
# Define callback methods which are called when events occur
# pylint: disable=unused-argument, redefined-outer-name
def connect(mqtt_client, userdata, flags, rc):
    # This function will be called when the mqtt_client is connected
    # successfully to the broker.
    print("Connected to MQTT Broker!")
    print("Flags: {0}\n RC: {1}".format(flags, rc))


def disconnect(mqtt_client, userdata, rc):
    # This method is called when the mqtt_client disconnects
    # from the broker.
    print("Disconnected from MQTT Broker!")


def subscribe(mqtt_client, userdata, topic, granted_qos):
    # This method is called when the mqtt_client subscribes to a new feed.
    print("Subscribed to {0} with QOS level {1}".format(topic, granted_qos))


def unsubscribe(mqtt_client, userdata, topic, pid):
    # This method is called when the mqtt_client unsubscribes from a feed.
    print("Unsubscribed from {0} with PID {1}".format(topic, pid))


def publish(mqtt_client, userdata, topic, pid):
    # This method is called when the mqtt_client publishes data to a feed.
    print("Published to {0} with PID {1}".format(topic, pid))


def message(client, topic, message):
    # Method called when a client's subscribed feed has a new value.
    print("New message on topic {0}: {1}".format(topic, message))


# Create a socket pool
pool = socketpool.SocketPool(wifi.radio)

# Set up a MiniMQTT Client
mqtt_client = MQTT.MQTT(
    broker=secrets["broker"],
    port=secrets["broker_port"],
#     username=secrets["broker_user"],
#     password=secrets["broker_pass"],
    socket_pool=pool,
#     ssl_context=ssl.create_default_context(),
)

# Connect callback handlers to mqtt_client
mqtt_client.on_connect = connect
mqtt_client.on_disconnect = disconnect
mqtt_client.on_subscribe = subscribe
mqtt_client.on_unsubscribe = unsubscribe
mqtt_client.on_publish = publish
mqtt_client.on_message = message

print("Attempting to connect to %s" % mqtt_client.broker)
mqtt_client.connect()

# print("Subscribing to %s" % co2_topic)
# mqtt_client.subscribe(co2_topic)

# print("Publishing to %s" % co2_topic)
# mqtt_client.publish(co2_topic, "Hello Broker!")

# print("Unsubscribing from %s" % co2_topic)
# mqtt_client.unsubscribe(co2_topic)

# print("Disconnecting from %s" % mqtt_client.broker)
# mqtt_client.disconnect()

i2c = busio.I2C(scl=board.SCL1, sda=board.SDA1, frequency=100000)
scd4x = adafruit_scd4x.SCD4X(i2c)
reset_pin = None

pm25 = PM25_I2C(i2c, reset_pin)
print("Found PM2.5 sensor, reading data..")

print("Serial number:", [hex(i) for i in scd4x.serial_number])

scd4x.start_periodic_measurement()
print("Waiting for first SCD41 measurement....")

run = True
while run:
    if scd4x.data_ready:
        print("CO2: %d ppm" % scd4x.CO2)
        print("Publishing to %s" % co2_topic)
        mqtt_client.publish(co2_topic, scd4x.CO2)

        print("Temperature: %0.1f *C" % scd4x.temperature)
        print("Publishing to %s" % temperature_topic)
        mqtt_client.publish(temperature_topic, scd4x.temperature)

        print("Humidity: %0.1f %%" % scd4x.relative_humidity)
        print("Publishing to %s" % humidity_topic)
        mqtt_client.publish(humidity_topic, scd4x.relative_humidity)
        print()

    try:
        aqdata = pm25.read()
        print(aqdata)
    except RuntimeError:
        print("Unable to read from sensor, retrying...")
        continue

    print()
    print("Concentration Units (standard)")
    print("---------------------------------------")
    print(
        "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
        % (aqdata["pm10 standard"], aqdata["pm25 standard"], aqdata["pm100 standard"])
    )
    mqtt_client.publish(pm10_standard_topic, aqdata["pm10 standard"])
    mqtt_client.publish(pm25_standard_topic, aqdata["pm25 standard"])
    mqtt_client.publish(pm100_standard_topic, aqdata["pm100 standard"])

    print("Concentration Units (environmental)")
    print("---------------------------------------")
    print(
        "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
        % (aqdata["pm10 env"], aqdata["pm25 env"], aqdata["pm100 env"])
    )
    print("---------------------------------------")
    mqtt_client.publish(pm10_env_topic, aqdata["pm10 env"])
    mqtt_client.publish(pm25_env_topic, aqdata["pm25 env"])
    mqtt_client.publish(pm100_env_topic, aqdata["pm100 env"])

    print("Particles > 0.3um / 0.1L air:", aqdata["particles 03um"])
    print("Particles > 0.5um / 0.1L air:", aqdata["particles 05um"])
    print("Particles > 1.0um / 0.1L air:", aqdata["particles 10um"])
    print("Particles > 2.5um / 0.1L air:", aqdata["particles 25um"])
    print("Particles > 5.0um / 0.1L air:", aqdata["particles 50um"])
    print("Particles > 10 um / 0.1L air:", aqdata["particles 100um"])
    print("---------------------------------------")
    mqtt_client.publish(particles_03um_topic, aqdata["particles 03um"])
    mqtt_client.publish(particles_05um_topic, aqdata["particles 05um"])
    mqtt_client.publish(particles_10um_topic, aqdata["particles 10um"])
    mqtt_client.publish(particles_25um_topic, aqdata["particles 25um"])
    mqtt_client.publish(particles_50um_topic, aqdata["particles 50um"])
    mqtt_client.publish(particles_100um_topic, aqdata["particles 100um"])
    time.sleep(10)
