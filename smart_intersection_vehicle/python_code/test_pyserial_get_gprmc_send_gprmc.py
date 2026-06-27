import serial
import datetime
import string
import time
import calendar 

def calculate_checksum(sentence):
    # Initialize the checksum
    checksum = 0

    # Iterate through each character in the sentence, excluding the leading '$'
    for char in sentence[1:]:
        # XOR each character's ASCII value with the current checksum
        checksum ^= ord(char)

    # Convert the checksum to a hexadecimal string
    checksum_hex = hex(checksum)[2:].upper()

    # Ensure the checksum is two characters long, prepend '0' if necessary
    if len(checksum_hex) == 1:
        checksum_hex = '0' + checksum_hex

    return checksum_hex

# # Original GPRMC signal
# original_sentence = "$GPRMC,180330,A,3404.1416,N,11826.5881,W,000.0,270.7,211023,012.4,E*62"

# # Split the sentence into parts
# parts = original_sentence.split(',')

# # Calculate the new checksum for the modified sentence
# new_checksum = calculate_checksum(','.join(parts[1:-1]))  # Exclude the '$' and '*62' parts

# # Modify the sentence with the new checksum and 'A' after 'E,'
# modified_sentence = f"${','.join(parts[1:-1])},{new_checksum},E,A*{new_checksum}"


if __name__ == '__main__':
    ser = serial.Serial("/dev/ttyUSB0", baudrate=9600)
    ser.flushInput()
    ser.flushOutput()
    idx = 0

    nmea_data = b""

    # skip first line, since it could be incomplete
    ser.readline()

    while True:
        idx += 1
        
        nmea_sentence = ser.readline()
        # print("dix",idx)
        print("origin:",nmea_sentence)
        # if its is in python2
        nmea_sentence = str(nmea_sentence).split('\'')
        nmea_sentence = nmea_sentence[1].split('\\')
        nmea_sentence = nmea_sentence[0]
        print(nmea_sentence)

        if "GPRMC" in (nmea_sentence):
            gprmc_data_plain = nmea_sentence
            # gprmc_data = pynmea2.parse(nmea_sentence)
            # print("gprmc_hms:",gprmc_data.datestamp)
            # print("gprmc_date:",gprmc_data.timestamp)
            # datetimestamp = utc_to_local(gprmc_data.datestamp,gprmc_data.timestamp)
            print(gprmc_data_plain)
            # convert the following string 
            # $GPRMC,002339,V,3404.1199,N,11826.6271,W,,,240223,012.4,E,A*77
            # into following format:
            # $GPRMC,224750.00,A,3404.13374332,N,11826.61263080,W,0.019,41.3,230223,11.3,E,A*29

            # Split the input string into two parts: the data before the checksum and the checksum itself
            parts = gprmc_data_plain.split("*")
            # parts = gprmc_data_plain.split(",01")
            data = parts[0]
            checksum = parts[1]
            # checksum_part = checksum.split("*")
            # c1 = checksum_part[0]
            # c2 = checksum_part[1]

            # Split the sentence into parts
            # parts = gprmc_data_plain.split(',')

            # # Calculate the new checksum for the modified sentence
            # new_checksum = calculate_checksum(','.join(parts[1:-1]))  # Exclude the '$' and '*62' parts

            # # Modify the sentence with the new checksum and 'A' after 'E,'
            # modified_sentence = f"${','.join(parts[1:-1])},{new_checksum},E,A*{new_checksum}"
            
            # print("checksum:",checksum)
            # Concatenate the data with the "A" character and the checksum, separated by "*"
            # output_str = data + ",A*" + checksum # this is wrong 
            output_str = data + ",,*" + checksum
            # output_str = data + ",1" + c1 + ",*" + c2

            print("OUTPUT:",output_str)
            # ser.write(gprmc_data_plain.encode())
            ser.write(output_str.encode())
            # nmea_data += nmea_sentence

    ser.close()





