# CODE: WRITE DATA TO CSV FILE

# importing libraries ||
import csv

# Global Variables ||


# Creating Functions ||
def Create_CSV(f, header):
    csv_writer = csv.writer(f)
    csv_writer.writerow(header)
    return csv_writer

def Write_To_CSV(csv_writer, row):
    csv_writer.writerow(row)


# Code Start ||


# For Testing ||
if __name__ == '__main__':

    try:
        filename = 'CSV_Temp.csv'
        header = ['ONE', 'TWO', 'THREE', 'FOUR', 'FIVE', 'SIX', 'SEVEN', 'EIGHT', 'NINE', 'TEN']
        f = open(filename, 'w', newline='')
        csv_writer = Create_CSV(f, header)
        value1 = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        value2 = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
        value3 = [6.7, 6.7, 6, 6, 6.7, 6, 6.7, 6, 6.7, 6]
        value4 = [4, 4, 4, 4, 4, 4, 4, 4, 4, 4]
        Write_To_CSV(csv_writer, value1)
        Write_To_CSV(csv_writer, value2)
        Write_To_CSV(csv_writer, value3)
        Write_To_CSV(csv_writer, value4)
    finally:
        f.close()  # close csv file
        print("FINISHING PROGRAM")
