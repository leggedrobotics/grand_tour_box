import numpy as np

# Read the file
with open("/home/jonfrey/git/boxi_paper/data_stats/2025-01-23_benchmark/cpu_usage_jetson.log", "r") as file:
    lines = file.readlines()

# Initialize lists to store %user and %system values
user_values = []
system_values = []

# Iterate through the lines and extract the relevant columns
for line in lines[3:]:
    if line.startswith("07:"):  # Check if the line contains data
        parts = line.split()
        user_values.append(float(parts[3].replace(",", ".")))  # Convert to float and replace comma with dot
        system_values.append(float(parts[5].replace(",", ".")))  # Convert to float and replace comma with dot

# Convert lists to NumPy arrays
user_array = np.array(user_values)
system_array = np.array(system_values)

# Combine the arrays into a single 2D array
combined_array = np.column_stack((user_array, system_array))

# Print the resulting array
print(combined_array.mean(axis=0))
