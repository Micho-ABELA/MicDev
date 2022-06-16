#!/usr/bin/env python3
# CODE: MAIN CODE FOR PI SLAVE | READS SENSORS AND SEND DATA TO PI MASTER

# importing libraries ||
from airplane_slave import Setup_Slave
from airplane_slave import Translator
from airplane_slave import Socket_Slave
from airplane_slave import SMBus
import logging
import socket
import pickle
import time
import os
import rospy
import rospkg

# Global Variables ||
config_file = os.path.join(rospkg.RosPack().get_path('airplane_slave'), 'src/airplane_slave', 'Slave_Config.ini')  # Config file
logging.basicConfig(filename=os.path.join(rospkg.RosPack().get_path('airplane_slave'), 'src/airplane_slave', 'Slave.log'),
                    level=logging.ERROR, format='%(levelname)s: %(message)s')  # config for Logger --> Slave.log file
lang_file = os.path.join(rospkg.RosPack().get_path('airplane_slave'), 'src/airplane_slave', 'Sentences.ini')  # Language file
bus = []  # List of BMP180 in the SMBus
temperature = []  # List to store temp data
pressure = []  # List to store pressure data
altitude = []  # List to store altitude data
pickles = []  # List to store all sensors data
buffer_size = 1024  # size of buffer for reception
END = '0'  # to stop program and shutdown Pi Slave
socket_connection_status = False  # to keep track of socket connectivity with Server


# Creating Functions ||
def CheckUp(lang, sensor):
    if lang != 'FR' and lang != 'EN' and lang != 'DE' and lang != 'IT':  # check language
        logging.error("check language in Slave_Config.ini file (wrong value)")
        return False
    for element in sensor:  # check the sensors list
        if element < 0 or element > 127:
            logging.error("the sensor ({}) has an incorrect address given in Slave_Config.ini file (wrong value)".format(hex(element)))
            return False
    return True


# Code Start ||

logging.error("///////////////////// new session //////////////////////////")  # Start new section in Log file

parameters, sensors = Setup_Slave.Read_File(config_file)  # Read config file and get data entered by user
host, port, lang, measuring_interval, measuring_number = Setup_Slave.Extract_Parameters(parameters)  # Extract data from list

check = CheckUp(lang, sensors)  # Check all user data
if not check:
    logging.error("The program stopped because you entered a wrong value in Slave_Config.ini, Please read above and fix it")
    END = '1'  # to stop program and shutdown Pi Slave
    quit()  # end program

sentences = Translator.Extract_Lines(lang_file, lang)  # get the sentences depending on language of user

try:
    rospy.init_node('PiSlave_node', anonymous=True)  # initialise Node for ROS

    s = Socket_Slave.Setup_Connection()  # create socket
    s.connect((host, port))  # attempt to connect to server, timeout of 20 seconds
    socket_connection_status = True  # to warn socket.timeout that connection to server is active

    for sensor in sensors:  # register each sensor in the SMBus
        bus.append(SMBus.BMP180.BMP085(address=sensor))

    while True:

        for sensor in bus:  # get Temperature, Pressure and Altitude for each sensor in SMBus
            temp, press, alt = SMBus.Read_Sensor(sensor, measuring_number)  # read sensor and get averaged values
            temperature.append(temp)
            pressure.append(press)
            altitude.append(alt)
        if len(temperature) == 0 or len(pressure) == 0 or len(altitude) == 0:  # check list of data before sending
            logging.error("list of value is empty (Temp/Pressure/Altitude), couldn't send data to Server")
            END = '1'  # to stop program and shutdown Pi Slave
            quit()  # end program

        pickles.append(pickle.dumps(temperature))  # dumps temp data
        pickles.append(pickle.dumps(pressure))  # dumps pressure data
        pickles.append(pickle.dumps(altitude))  # dumps altitude data

        for data in pickles:  # send all data to Raspberry Pi Server
            Socket_Slave.Send_Pickle(data, s)  # send list of values via Socket
            response = Socket_Slave.Recv_Data(buffer_size, s)  # wait for the OK from Server (BLOCKING)
            if response != 'OK' and response != '1':
                logging.error("couldn't send data to Raspberry Pi Server, due to the {} element".format(pickles.index(data)))
                END = '1'  # to stop program and shutdown Pi Slave
                quit()  # end program
            if response == '1':  # case: receive caught the exit program
                logging.error("RECEIVED ORDER PROPERLY, SHUTTING DOWN THE PI...")
                END = '1'  # to stop program and shutdown Pi Slave
                quit()  # end program

        temperature.clear()  # empty temp data
        pressure.clear()  # empty pressure data
        altitude.clear()  # empty altitude data
        pickles.clear()  # empty pickles data to send a new one

        END = Socket_Slave.Recv_Data(buffer_size, s)  # check if user wants to exit code
        if END == '1':
            logging.error("RECEIVED ORDER PROPERLY, SHUTTING DOWN THE PI...")
            quit()  # end program

        time.sleep(measuring_interval)  # wait a bit before measuring again

except socket.timeout:  # in case: failed connection with server (Pi Master)
    logging.error("couldn't connect to Raspberry Pi Server (host= {}, port= {}), Please change host or port in Slave_Config.ini file".format(host, port))
    if socket_connection_status:
        END = '1'  # to stop program and shutdown Pi Slave
        quit()  # end program
except pickle.PicklingError:  # in case: failed to pickle an object
    logging.error("couldn't pickle an object... problem while sending lists to Raspberry Pi Server")
    END = '1'  # to stop program and shutdown Pi Slave
    quit()  # end program
except rospy.ROSInterruptException:  # case: ros interruption
    END = '1'  # to stop program and shutdown Pi Slave
finally:
    s.close()  # close socket connection
    logging.error("EXIT Program... OK")
    if END == '1':  # case: the user stopped using the program, in case he wanted Modifications
        logging.error("SHUTDOWN... OK")
        os.system('sudo shutdown now')  # shutdown the Pi Slave
