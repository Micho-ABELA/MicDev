#!/usr/bin/env python3
# CODE: MAIN CODE FOR PI MASTER | GET THE SENSOR DATA FROM SLAVE AND PRINT ON GUI

# importing libraries ||
from airplane_master import Socket_Master
from airplane_master import Setup_Master
from airplane_master import CSV_Master
from airplane_master import Translator
import time
import logging
import os
import socket
import rospy
import rospkg
from airplane_master.msg import custom
from std_msgs.msg import String


# Global Variables ||
config_file = os.path.join(rospkg.RosPack().get_path('airplane_master'), 'src/airplane_master', 'Master_Config.ini')  # Config file
lang_file = os.path.join(rospkg.RosPack().get_path('airplane_master'), 'src/airplane_master', 'Sentences.ini')  # Sentences File
logging.basicConfig(filename=os.path.join(rospkg.RosPack().get_path('airplane_master'), 'src/airplane_master', 'Master.log')
                    , level=logging.ERROR, format='%(levelname)s: %(message)s') # To log information --> Master.log file
temperature = []  # List to store temp data
pressure = []  # List to store pressure data
altitude = []  # List to store altitude data
pickles = []  # To store all pickle data
buffer_size = 1024  # Buffer size for socket communication
Quit = "0"  # Variable to stop program from GUI (user)
Record = False  # Variable to toggle between Start and Stop recording
response = 'OK'  # response to Slave
csv_temp = os.path.join(rospkg.RosPack().get_path('airplane_master'), 'src/airplane_master', 'CSV_Temp.csv')  # temp csv file
csv_pres = os.path.join(rospkg.RosPack().get_path('airplane_master'), 'src/airplane_master', 'CSV_Pressure.csv')  # pressure csv file
csv_alt = os.path.join(rospkg.RosPack().get_path('airplane_master'), 'src/airplane_master', 'CSV_Altitude.csv')  # altitude csv file
header = ['ONE', 'TWO', 'THREE', 'FOUR', 'FIVE', 'SIX', 'SEVEN', 'EIGHT', 'NINE', 'TEN']  # header of CSV files
socket_connection_status = False  # to keep track of socket connectivity with Server
copy_destination = '/var/www/html/CSV/'  # copy the CSV files to this directory, so the PiMaster can host them for the user


# Creating Functions ||
def CheckUp(lang):
    if lang != 'FR' and lang != 'EN' and lang != 'DE' and lang != 'IT':  # check language
        logging.error("check language in Master_Config.ini file (wrong value)")
        return False
    return True

def quit_callback(data):
    global Quit
    Quit = data.data

def record_callback(data_record):
    global Record, f1, f2, f3, temp_writer, pres_writer, alt_writer
    if data_record.data == "start":  # signal to start recording
        Record = True
    elif data_record.data == "stop":  # signal to stop recording
        Record = False
        f1.close()
        f2.close()
        f3.close()
        # COPY CSV FILES HERE TO APACHE
        time_now = CSV_Master.Get_Unique_Name()  # get time to save CSV files with unique names
        os.system('echo pi_master | sudo -S cp {} {}CSV_Temp{}.csv'.format(csv_temp, copy_destination, time_now))  # copy Temperature CSV into Apache's host
        os.system('echo pi_master | sudo -S cp {} {}CSV_Press{}.csv'.format(csv_pres, copy_destination, time_now))  # copy Pressure CSV into Apache's host
        os.system('echo pi_master | sudo -S cp {} {}CSV_Alti{}.csv'.format(csv_alt, copy_destination, time_now))  # copy Altitude CSV into Apache's host
        # Reopen CSV files to clear/empty them
        f1 = open(csv_temp, 'w', newline='')  # open temp csv file
        f2 = open(csv_pres, 'w', newline='')  # open pressure csv file
        f3 = open(csv_alt, 'w', newline='')  # open altitude csv file
        temp_writer = CSV_Master.Create_CSV(f1, header)
        pres_writer = CSV_Master.Create_CSV(f2, header)
        alt_writer = CSV_Master.Create_CSV(f3, header)


# Code Start ||

logging.error("///////////////////// new session //////////////////////////")  # To start a new session in Log file

os.system('echo pi_master | sudo -S ip ad add 10.0.0.10/24 dev eth0')  # Assign ip address for eth0 of PiMaster

parameters, names = Setup_Master.Read_File(config_file)  # Get the parameters entered by user in config file
host, port, lang, measuring_interval = Setup_Master.Extract_Parameters(parameters)  # Extract values from the list

check = CheckUp(lang)  # Check language variable
if not check:
    logging.error("The program stopped because you entered a wrong value in Master_Config.ini, Please read above and fix it")
    quit()  # end program

sentences = Translator.Extract_Lines(lang_file, lang)  # get the sentences from file

try:
    rospy.init_node('PiMaster_node', anonymous=True)  # initialise Node for ROS
    pub = rospy.Publisher('data_topic', custom, queue_size=10)  # create publisher to send data to GUI
    DATA = custom()  # this is the data type to send Temp, Pressure, Altitude and Connection_status
    rospy.Subscriber('topic_quit', String, quit_callback)  # listen to the QUIT button from GUI
    rospy.Subscriber('topic_record', String, record_callback)  # listen to the Record button from GUI

    s = Socket_Master.Setup_Server(host, port)  # Create the socket Server
    client, address = s.accept()  # accept the client connection (Pi Slave)
    socket_connection_status = True  # to warn socket.timeout that client is now connected

    f1 = open(csv_temp, 'w', newline='')  # open temp csv file
    f2 = open(csv_pres, 'w', newline='')  # open pressure csv file
    f3 = open(csv_alt, 'w', newline='')  # open altitude csv file
    temp_writer = CSV_Master.Create_CSV(f1, header)
    pres_writer = CSV_Master.Create_CSV(f2, header)
    alt_writer = CSV_Master.Create_CSV(f3, header)

    time.sleep(5)  # wait 5 seconds for client to Read Sensors

    while True:

        for i in range(3):  # loop 3 times to get all the Temp/Pressure/Altitude data
            data = Socket_Master.Recv_Pickle(client, buffer_size)  # Receive the data from Slave
            pickles.append(data)  # add it to the pickles list
            Socket_Master.Send_Data(client, response)  # Send the OK to the Slave to continue

        Socket_Master.Send_Data(client, Quit)  # Send Quit data to Slave

        temp = pickles[0]  # Temporary temp data
        pres = pickles[1]  # Temporary pressure data
        alt = pickles[2]  # Temporary altitude data

        for element in temp:
            temperature.append(round(element, 1))  # filter values to 1 decimal
        for element in pres:
            pressure.append(round(element, 1))  # filter values to 1 decimal
        for element in alt:
            altitude.append(round(element, 1))  # filter values to 1 decimal

        # here send to GUI
        DATA.temperature_list = temperature
        DATA.pressure_list = pressure
        DATA.altitude_list = altitude
        DATA.connection_status = True
        DATA.sensor_names = names
        pub.publish(DATA)  # publish the message to the topic

        if Record:  # if user wants to record the data passing by
            CSV_Master.Write_To_CSV(temp_writer, temperature)  # write temp to CSV
            CSV_Master.Write_To_CSV(pres_writer, pressure)  # write pressure to CSV
            CSV_Master.Write_To_CSV(alt_writer, altitude)  # write altitude to CSV

        pickles.clear()  # clear data
        temperature.clear()  # clear data
        pressure.clear()  # clear data
        altitude.clear()  # clear data
        temp.clear()  # clear data
        pres.clear()  # clear data
        alt.clear()  # clear data

        if Quit == "1":  # to Exit program
            Socket_Master.Send_Data(client, Quit)  # send Force Exit to Slave
            break

        time.sleep(measuring_interval)  # wait a bit before reading again

except socket.timeout:  # case: a socket timeout is raised
    logging.error("couldn't connect to Raspberry Pi Slave (timeout)")
    if socket_connection_status:
        quit()  # end program
except rospy.ROSInterruptException:  # case: ros interruption
    quit()  # end program
finally:
    s.close()
    if socket_connection_status:  # close CSV Files manager
        f1.close()
        f2.close()
        f3.close()
        if Record:  # if user stopped program without saving his record
            time_now = CSV_Master.Get_Unique_Name()  # get time to save CSV files with unique names
            os.system('echo pi_master | sudo -S cp {} {}CSV_Temp{}.csv'.format(csv_temp, copy_destination, time_now))  # copy Temperature CSV into Apache's host
            os.system('echo pi_master | sudo -S cp {} {}CSV_Press{}.csv'.format(csv_pres, copy_destination, time_now))  # copy Pressure CSV into Apache's host
            os.system('echo pi_master | sudo -S cp {} {}CSV_Alti{}.csv'.format(csv_alt, copy_destination, time_now))  # copy Altitude CSV into Apache's host
    logging.error("EXIT Program... OK")
