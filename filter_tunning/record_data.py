import serial
import csv
import statistics

# Configure constants and data names
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
DATA_LIST = ['left', 'right']
DATA_NAME = 'data'

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Open CSV file for writing
with open(f'{DATA_NAME}.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(DATA_LIST) # write headers to CSV file
    
    # Initialize lists for storing data
    data_lists = {val: [] for val in DATA_LIST}
    
    try:
        # Continuously read data from serial and write to CSV file
        while True:
            data = ser.readline().decode().strip()
            data_values = data.split()
            if len(data_values) == len(DATA_LIST):
                try:
                    data_dict = dict(zip(DATA_LIST, map(float, data_values)))
                    if all(data_dict[val] < 1 for val in DATA_LIST):
                        # Write data to CSV file
                        writer.writerow([data_dict[val] for val in DATA_LIST])
                        csvfile.flush()
                        
                        # Append data to lists
                        for val in DATA_LIST:
                            data_lists[val].append(data_dict[val])
                except ValueError:
                    pass
    except KeyboardInterrupt:
        # Calculate mean and standard deviation of data
        stats_list = [['data', 'mean', 'stddev']]
        for val in DATA_LIST:
            data_mean = statistics.mean(data_lists[val])
            data_stddev = statistics.stdev(data_lists[val])
            stats_list.append([val, data_mean, data_stddev])
        
        # Write mean and standard deviation to CSV file
        with open(f'{DATA_NAME}_stats.csv', 'w', newline='') as statsfile:
            writer = csv.writer(statsfile, delimiter=',')
            for row in stats_list:
                writer.writerow(row)
        
        # Print mean and standard deviation of data
        for val in DATA_LIST:
            data_mean = statistics.mean(data_lists[val])
            data_stddev = statistics.stdev(data_lists[val])
            print(f"{val} mean: {data_mean}, {val} std. deviation: {data_stddev}")
