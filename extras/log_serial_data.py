# Quick sketch that stores serial data in a text file
#
# Author: Juan Gallostra
# Date: 16-10-2018

import serial

# Serial parameters
PORT = '/dev/ttyACM0'
BAUDRATE = 115200
LOG_DURATION = 60

def store_sensor_data(serial_obj, text_file):
	"""
	Get data from serial. Modify function as needed.
	"""
	# read serial data and get the different values received
	raw_data = serial_obj.readline().rstrip().split(",")
	data = list(map(float, raw_data))
	# print to terminal so that one can see what's being stored
	print data 
	# Store data in the specified file
	text_file.write(",".join(map(str, data)) + "\n")
	# data[0] should be the timestamp of the readings
	return data[0] # should return time


def main(port=PORT, baudrate=BAUDRATE, duration=LOG_DURATION):
	# Serial communication object
	serial_com = serial.Serial(port, baudrate)

	first_iter = True
	time_elapsed = False
	starting_time = 0 

	with open("dataset.csv", "w") as f:
		while not time_elapsed:	
        		current_time = store_sensor_data(serial_com, f)
        		if first_iter:
            			starting_time = current_time
            			first_iter = False
        		if current_time - starting_time > duration:
            			time_elapsed = True

	serial_com.close()

if __name__ == "__main__":
	main()
