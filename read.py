with open("/dev/aesd_bme_device") as device_file:
    strings = [line for line in device_file.readlines()]
    if(len(strings)!=3):
        print("Unexpected response from device file")
        exit(42)
    temp_s = strings[0]
    hum_s = strings[1]
    pres_s = strings[2]
    print(f"Temperature:\t\t{int(temp_s)/100} °C")
    print(f"Humidity:\t\t{int(hum_s)/1024} %rel")
    print(f"Pressure:\t\t{int(pres_s)/256} hPa")
