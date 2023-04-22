import numpy as np

DATA_NAME = 'data'

# Load the data from the CSV file with header
data = np.genfromtxt(f'{DATA_NAME}.csv', delimiter=',', names=True)

for data_name in data.dtype.names:
    # Extract the values for the given column
    values = data[data_name]

    # Sort the values in ascending order
    sorted_values = np.sort(values)

    # Find the N smallest and N largest values
    N = len(values) // 100
    smallest_N = sorted_values[:N]
    largest_N = sorted_values[-N:]

    # Calculate the average of the N smallest and largest values
    avg_smallest_N = np.mean(smallest_N)
    avg_largest_N = np.mean(largest_N)

    print(f"{data_name} min: {avg_smallest_N}, {data_name} max: {avg_largest_N}")
    