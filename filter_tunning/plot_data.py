import numpy as np
import matplotlib.pyplot as plt

DATA_NAME = 'data'
INTERESTED_DATA = ['FL1', 'FR1']

# Load the data from the CSV file with header
data = np.genfromtxt(f'{DATA_NAME}.csv', delimiter=',', names=True)
data_stats = np.genfromtxt(f'{DATA_NAME}_stats.csv', delimiter=',', names=True)

# Plot the original data and the filtered data
plt.figure(figsize=(12, 6))
for i, data_name in enumerate(data.dtype.names):
    if data_name in INTERESTED_DATA:
        plt.plot(
            data[data_name],
            label=
            f'{data_name} (mean: {data_stats["mean"][i]:.3f}, stddev: {data_stats["stddev"][i]:.3f})'
        )
plt.legend()
plt.title('Recorded Data')
plt.xlabel('Time')
plt.ylabel('Values')
plt.show()
