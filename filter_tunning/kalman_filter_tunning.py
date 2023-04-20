import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import KalmanFilter1D

# Load the data from the CSV file with header
data_file = np.genfromtxt('data.csv', delimiter=',', names=True)
data = data_file['data']

# Load the stats from the CSV file with header
stats_file = np.genfromtxt('stats.csv', delimiter=',', names=True)
mean = stats_file['mean']
std = stats_file['std']

# Create a Kalman Filter with the given parameters
Q = 0.01
R = std ** 2

kalman_filter = KalmanFilter1D(Q, R, mean, std)

filtered_data = np.empty_like(data)
for i in range(len(data)):
    filtered_data[i] = kalman_filter.update(data[i])

# Plot the original data and the filtered data
plt.figure(figsize=(12, 6))
plt.plot(data, label='Original Data')
plt.plot(filtered_data, label=f'Filtered Data (Kalman Filter, Q={Q})')
plt.legend()
plt.title('Original Data vs Filtered Data with Kalman Filter')
plt.xlabel('Time')
plt.ylabel('Values')
plt.show()
