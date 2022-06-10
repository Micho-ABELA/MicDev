# CODE: READS THE SENSORS AND RETURNS A LIST OF AVERAGED VALUES

# importing libraries ||
# from smbus2 import SMBus
# import time
from airplane_slave import BMP085 as BMP180


# Global Variables ||
# measuring_number = 5
# measuring_interval = 0.3


# def Data_Sensors(bus, addresses):
#    sensors = []
#    for sensor in addresses:
#        sensors.append(Read_Sensors(bus, sensor))
#    return sensors

# def Read_Sensors(bus, address):
#    values = []
#    Sum = 0
#    for i in range(measuring_number):
#        values.append(bus.read_byte_data(address, ))  # address, register, force
#        Sum += values[i]
#        time.sleep(measuring_interval)
#    data = Sum / measuring_number
#    return data

# Creating Functions ||

def Read_Sensor(bus, measuring_number):
    temp = []
    pressure = []
    altitude = []
    temp_sum = 0
    pressure_sum = 0
    altitude_sum = 0
    for i in range(measuring_number):
        temp.append(bus.read_temperature())  # read Temperature C (float)
        pressure.append(bus.read_pressure())  # read Pressure Pa (int)
        altitude.append(bus.read_altitude())  # read Altitude M (float)
        temp_sum += temp[i]
        pressure_sum += pressure[i]
        altitude_sum += altitude[i]
    Temp = temp_sum / measuring_number
    Pressure = pressure_sum / measuring_number
    Altitude = altitude_sum / measuring_number
    return Temp, Pressure, Altitude


# Code Start ||


# For Testing ||
if __name__ == "__main__":

    try:
        sensor = BMP180.BMP085(1, 112)
        temp, pressure, altitude = Read_Sensor(sensor, 5)
        print("Temperature: {} Celsius".format(temp))
        print("Pressure: {} Pa".format(pressure))
        print("Altitude: {} Meters".format(altitude))
    finally:
        print("FINISHING PROGRAM")




