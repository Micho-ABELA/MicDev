# CODE: GETS ALL THE PARAMETERS ENTERED BY THE USER IN A CONFIG FILE

# importing libraries ||
from configparser import ConfigParser

# Global Variables ||


# Creating Functions ||
def Read_File(file):
    parameters = []
    sensors = []
    config = ConfigParser()
    config.read(file)
    for section in config.sections():
        if section == 'Sensors':
            for element in config[section]:
                sensors.append(int(config[section][element], 16))
        else:
            for element in config[section]:
                parameters.append(config[section][element])
    return parameters, sensors

def Extract_Parameters(list):
    host = list[0]
    port = int(list[1])
    language = list[2]
    interval = int(list[3])
    number = int(list[4])
    return host, port, language, interval, number

def Write_to_File(file, section=None, element=None, value=None):
    config = ConfigParser()
    config.read(file)
    if not config.has_section(section):
        if section is None:
            return
        else:
            config.add_section(section)
            config.set(section, element, value)
    if config.has_section(section):
        config.set(section, element, value)
    with open(file, 'w') as Conf:  # Write changes to config file
        config.write(Conf)


# Code Start ||


# For Testing ||
if __name__ == "__main__":

    try:
        MyFile = 'Slave_Config.ini'
        Parameters, Sensors = Read_File(MyFile)  # returns a list of parameters entered by the user
        host, port, lang, measuring_interval, measuring_number = Extract_Parameters(Parameters)
        print("The Host is: {}".format(host))
        print("The Port is: {}".format(port))
        print("The Language is: {}".format(lang))
        print("The Interval is: {}".format(measuring_interval))
        print("The Number is: {}".format(measuring_number))
        print("The list of sensor: {}".format(Sensors))
        print("The Sensor is: {}".format(type(Sensors[0])))
        # Write_to_File(MyFile, 'socket', 'host', '192.168.4.142')
        # Write_to_File(MyFile, 'socket', 'haha', '192')
    finally:
        print("FINISHING PROGRAM")
