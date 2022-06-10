# CODE: EXTRACT THE SENTENCES DEPENDING ON THE CHOSEN LANGUAGE

# Importing Libraries ||
from configparser import ConfigParser

# Global Variables ||


# Creating Functions ||
def Extract_Lines(file, language):
    sentences = []
    config = ConfigParser()
    config.read(file)
    for section in config.sections():
        if section == language:
            for element in config[section]:
                sentences.append(config[section][element])
    return sentences

# Code Start ||


# For Testing ||
if __name__ == "__main__":

    try:
        MySentences = Extract_Lines('Sentences.ini', 'EN')
        print(MySentences)
        print(MySentences[0])
        print(MySentences[1])
        print(MySentences[2])
        print(MySentences[3])
    finally:
        print("FINISHING PROGRAM")
