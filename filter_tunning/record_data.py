import numpy as np
import serial

serial = serial.Serial('/dev/ttyACM0', 115200)

data = np.empty(0)

try:
    while True:
        # Read the data from the serial port
        line = serial.readline().decode('utf-8').strip()
        # Append the data to the data array
        data = np.append(data, float(line))

finally:
    # Close the serial port
    serial.close()

    # Calculate the mean and standard deviation of the data
    mean = np.mean(data)
    std = np.std(data)
    stats = np.array([mean, std])
    print(f'Mean: {mean}, Standard Deviation: {std}')

    # Save the data to a CSV file with the header
    np.savetxt('data.csv', data, delimiter=',', header='data')

    # Save the stats to a CSV file with the header
    np.savetxt('stats.csv', stats.reshape(-1, 2), delimiter=',', header='mean,std')
