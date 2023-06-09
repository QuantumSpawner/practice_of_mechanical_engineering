import numpy as np
import matplotlib.pyplot as plt

from filter import FilterChain, NormalizeFilter, MovingAverageFilter

DATA_NAME = 'data'
INTERESTED_DATA = ['FL1', 'FR1']
UPPER_BOUND = [1, 1, 1, 1, 0.526, 0.542]
LOWER_BOUND = [0, 0, 0, 0, 0.033, 0.034]
WINDOW_SIZE = 10

# Load the data from the CSV file with header
data = np.genfromtxt(f'{DATA_NAME}.csv', delimiter=',', names=True)
data_stats = np.genfromtxt(f'{DATA_NAME}_stats.csv', delimiter=',', names=True)

# Create two subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))

for i, data_name in enumerate(data.dtype.names):
    if data_name in INTERESTED_DATA:
        # Calculate the filtered data
        moving_average_filter = MovingAverageFilter(WINDOW_SIZE)
        normalize_filter = NormalizeFilter(UPPER_BOUND[i], LOWER_BOUND[i])
        filter_chain = FilterChain([moving_average_filter, normalize_filter])

        filtered_data = np.empty_like(data[data_name])
        for j in range(len(filtered_data)):
            filtered_data[j] = filter_chain.update(data[data_name][j])
        filtered_mean = np.mean(filtered_data)
        filtered_stddev = np.std(filtered_data)

        # Plot the original data in the first subplot
        ax1.plot(
            data[data_name],
            label=
            f'{data_name} (mean: {data_stats["mean"][i]:.3f}, stddev: {data_stats["stddev"][i]:.3f})'
        )
        ax1.set_title('Original Data')
        ax1.set_xlabel('Time')
        ax1.set_ylabel('Values')

        # Plot the filtered data in the second subplot
        ax2.plot(
            filtered_data,
            label=
            f'{data_name} filtered (mean: {filtered_mean:.3f}, stddev: {filtered_stddev:.3f})'
        )
        ax2.set_title('Filtered Data')
        ax2.set_xlabel('Time')
        ax2.set_ylabel('Values')

# Set the legends for both subplots
ax1.legend()
ax2.legend()

# Show the subplots
plt.show()
