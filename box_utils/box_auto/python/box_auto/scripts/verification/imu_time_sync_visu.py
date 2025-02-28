import yaml
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Path to your YAML file (adjust this to your actual file path)
yaml_file_path = (
    "/media/jonfrey/Data/deployment_day_18/2024-12-09-11-28-28/verification/all_missions_summary_skip_30.yaml"
)

# Step 1: Load the YAML file
with open(yaml_file_path, "r") as file:
    data = yaml.safe_load(file)

# Step 2: Parse the data into a list of dictionaries
rows = []
for timestamp, topics in data.items():

    if len(topics) != 4:
        print(timestamp, " Topics Wrong: ", topics)
        continue
    try:
        if abs(topics["/gt_box/adis16475_node/imu"]) > 20 * 10**6:
            print(timestamp, " Sync Wrong: ", topics)
            print(5 * 10**6)
            continue
        if topics["/gt_box/stim320/imu"] < 20 * 10**6 and topics["/gt_box/stim320/imu"] > 10 * 10**6:
            print(topics["/gt_box/stim320/imu"], timestamp)

    except:
        continue

    topics = {k: float(v / 10**6) for k, v in topics.items()}
    row = {"timestamp": timestamp}
    row.update(topics)  # Add all topic values to the row
    rows.append(row)

# Step 3: Create a Pandas DataFrame
df = pd.DataFrame(rows)
df.set_index("timestamp", inplace=True)

# Step 4: Define plotting parameters
num_bins = 100
use_same_range = True  # Set to False to use individual min/max for each topic

# Step 5: Determine bin ranges
if use_same_range:
    global_min = df.min().min()  # Minimum value across all topics
    global_max = df.max().max()  # Maximum value across all topics
    bin_range = (global_min, global_max)
else:
    bin_range = None  # Matplotlib will determine range per topic

# Step 6: Create subplots (2x2 grid)
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
axes = axes.flatten()  # Flatten to easily iterate over

# List of topics (columns in the DataFrame)
topics = df.columns

# Step 7: Plot histograms
for i, topic in enumerate(topics):
    if bin_range:
        bins = np.linspace(bin_range[0], bin_range[1], num_bins + 1)
    else:
        bins = num_bins  # Let Matplotlib auto-determine range

    print(df[topic].min(), df[topic].max())
    axes[i].hist(df[topic], bins=bins, edgecolor="black")
    axes[i].set_title(topic.split("/")[-2])  # Simplified title (e.g., "adis16475_node")
    axes[i].set_xlabel("Value in ms")
    axes[i].set_ylabel("Frequency")

# Step 8: Adjust layout and display
plt.tight_layout()
plt.show()
