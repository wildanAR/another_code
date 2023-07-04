import psutil

def battery():
    batt = psutil.sensors_battery()
    battery = batt.percent
    charge = batt.power_plugged

    if charge is True:
        print('laptop lagi di cas')
        print(battery)

if __name__ == '__main__':
    battery()