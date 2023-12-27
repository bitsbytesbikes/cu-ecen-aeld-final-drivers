import json
import time
import serial
import io

hostname = ""
with open("/etc/hostname") as host_file:
    hostname = host_file.readlines()[0].strip()

ser = serial.Serial(
    port = '/dev/ttyACM0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
)

sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

while True:
    location = 'N/A'
    while not location.startswith("$GPRMC"):
        try:
            location = sio.readline()
        except serial.SerialException as e:
            print(f"Serial connection error: {e}" )
            break;
        except e:
            print(e)

    with open("/dev/aesd_bme_device") as device_file:
        strings = [line for line in device_file.readlines()]
        if(len(strings)!=3):
            print("Unexpected response from device file")
            exit(42)
        temp_s = strings[0]
        hum_s = strings[1]
        pres_s = strings[2]
        #print(f"Temperature:\t\t{int(temp_s)/100} °C")
        #print(f"Humidity:\t\t{int(hum_s)/1024} %rel")
        #print(f"Pressure:\t\t{int(pres_s)/256} hPa")
        json_data = {
            "location": location,
            "station": hostname,
            "temperature": int(temp_s)/100,
            "humidity": int(hum_s)/1024,
            "pressure": int(pres_s)/256
        }
        print(json.dumps(json_data))

