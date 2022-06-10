# CODE: GET THE PARAMETERS SET BY THE USER IN THE CONFIG FILE

# importing libraries ||
from configparser import ConfigParser

# Global Variables ||


# Creating Functions ||
def Read_File(file):
    parameters = []
    names = []
    config = ConfigParser()
    config.read(file)
    for section in config.sections():
        if section == 'Sensors':
            for element in config[section]:
                names.append(config[section][element])
        else:
            for element in config[section]:
                parameters.append(config[section][element])
    return parameters, names

def Extract_Parameters(list):
    host = list[0]
    port = int(list[1])
    language = list[2]
    interval = int(list[3])
    return host, port, language, interval

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
        MyFile = 'Master_Config.ini'
        Parameters = Read_File(MyFile)  # returns a list of parameters entered by the user
        host, port, lang, measuring_interval = Extract_Parameters(Parameters)
        print("The Host is: {}".format(host))
        print("The Port is: {}".format(port))
        print("The Language is: {}".format(lang))
        print("The Interval is: {}".format(measuring_interval))
        # Write_to_File(MyFile, 'socket', 'host', '192.168.4.142')
        # Write_to_File(MyFile, 'socket', 'haha', '192')
    finally:
        print("FINISHING PROGRAM")
