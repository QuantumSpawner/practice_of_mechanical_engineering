import numpy as np
import matplotlib.pyplot as plt

from filter import ExponentialFilter

DATA_NAME = 'data'
ALPHA = 0.1

# Load the data from the CSV file with header
data = np.genfromtxt(f'{DATA_NAME}.csv', delimiter=',', names=True)
data_stats = np.genfromtxt(f'{DATA_NAME}_stats.csv', delimiter=',', names=True)

# Plot the original data and the filtered data
plt.figure(figsize=(12, 6))
for i, data_name in enumerate(data.dtype.names):
    # Calculate the filtered data
    filter = ExponentialFilter(ALPHA)
    filtered_data = np.empty_like(data[data_name])
    for j in range(len(filtered_data)):
        filtered_data[j] = filter.update(data[data_name][j])
    filtered_mean = np.mean(filtered_data)
    filtered_stddev = np.std(filtered_data)

    plt.plot(data[data_name], label=f'{data_name} (mean: {data_stats["mean"][i]:.3f}, stddev: {data_stats["stddev"][i]:.3f})')
    plt.plot(filtered_data, label=f'{data_name} filtered (mean: {filtered_mean:.3f}, stddev: {filtered_stddev:.3f})')
plt.legend()
plt.title(f'Recorded Data vs Filtered Data with Exponenetial Filter (alpha: {ALPHA})')
plt.xlabel('Time')
plt.ylabel('Values')
plt.show()
