import os
import numpy as np
from evo.tools import file_interface
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams["text.usetex"] = True
mpl.rcParams["figure.facecolor"] = "white"  # All future figures
mpl.rcParams["axes.facecolor"] = "white"  # All axes backgrounds


# Define the function to generate filenames
def generate_filename(base, prefix, delay):
    # if delay == 0:
    #     return f"{base}{prefix}_tp_all.zip"
    # ms_value = abs(delay) / 1000  # Convert µs to ms
    # ms_str = f"{ms_value:.1f}".replace(".", "") if ms_value < 1 else f"{int(ms_value)}"
    # sign = "minus_" if delay < 0 else ""
    return f"{base}{prefix}_{delay}_tp_all.zip"


# Base directory paths
base_ape = "/home/tutuna/Videos/2024-11-03-13-51-43_eigergletscher_hike_down/2024-11-03-13-51-43_evo_evaluations/2024-11-03-13-51-43_ape_results/"
base_rpe = "/home/tutuna/Videos/2024-11-03-13-51-43_eigergletscher_hike_down/2024-11-03-13-51-43_evo_evaluations/2024-11-03-13-51-43_rpe_results/"
output_dir = base_ape
x = ["cpt7", "stim", "adis", "ap20", "alphasense", "livox"]  # , "zed2i"

# Generate file paths and load data
ape_results = {}
rpe_results = {}

for delay in x:
    filename_ape = generate_filename(base_ape, "ape_results_dlio", delay)
    filename_rpe = generate_filename(base_rpe, "RPE_results_dlio", delay)

    ape_results[delay] = file_interface.load_res_file(filename_ape, load_trajectories=True)
    rpe_results[delay] = file_interface.load_res_file(filename_rpe, load_trajectories=True)

# Initialize lists for statistics
y_ape = []
yerr_ape = []
y_rpe = []
yerr_rpe = []

# Extract statistics dynamically
for delay in x:
    ape_data = ape_results.get(delay, None)
    rpe_data = rpe_results.get(delay, None)

    if ape_data:
        y_ape.append(ape_data.stats["mean"])
        yerr_ape.append(ape_data.stats["std"])

    if rpe_data:
        y_rpe.append(rpe_data.stats["mean"])
        yerr_rpe.append(rpe_data.stats["std"])


# Create a figure with two subplots (1 row, 2 columns)
fig, axs = plt.subplots(2, 1, figsize=(16, 12))  # Increase figure width for horizontal layout

# Convert specific IMU types to their respective names
for i in range(len(x)):
    if x[i] == "cpt7":
        x[i] = "HG4930"
    elif x[i] == "adis":
        x[i] = "ADIS16475"
    elif x[i] == "stim":
        x[i] = "STIM320"
    elif x[i] == "ap20":
        x[i] = "AP20 IMU"
    elif x[i] == "alphasense":
        x[i] = "BMI085"
    elif x[i] == "livox":
        x[i] = "CM40609"


# ===============================
# First Subplot: APE
# ===============================

x_positions = np.arange(len(x))

axs[0].errorbar(
    x_positions,
    y_ape,
    yerr=yerr_ape,
    fmt="o",
    markersize=10,
    capsize=6,
    elinewidth=2,
    capthick=2,
    color="darkblue",
    label="Mean ± 0.5x Std Dev",
)

# axs[0].set_xlabel("Time Offset Value [μs]", fontsize=20, fontweight="bold")
axs[0].set_ylabel("APE [m]", fontsize=20, fontweight="bold")

# Format X-axis
axs[0].set_xticks(x_positions)
# xticks = axs[0].get_xticks()
# xticks_formatted = [int(tick) if ((np.abs(tick) >= 1) or (tick==0.0)) else  f".{str(tick)[2:]}" for tick in xticks]

# Apply formatted ticks
# axs[0].set_xticks(xticks)  # Set positions


# xtick_labels = [
#     str(int(tick)) if ((abs(tick) >= 1) or (tick==0.0) ) else f"{'-' if tick < 0 else ''}.{str(abs(tick))[2:]}"
#     for tick in xticks
# ]

axs[0].set_xticklabels(x)  # Set labels

# axs[0].set_xticks(x_plotting)
axs[0].tick_params(axis="both", labelsize=20)
# axs[0].ticklabel_format(axis="x", style="sci", scilimits=(0,0))
# axs[0].xaxis.set_major_locator(mticker.AutoLocator())  # Automatically choose appropriate integer ticks
# axs[0].xaxis.set_minor_locator(mticker.MultipleLocator(0.5))  # Add minor ticks at 0.5 intervals if needed

axs[0].set_xlim(min(x_positions) - 1, max(x_positions) + 1)
# Grid for better readability
axs[0].grid(True, linestyle="--", linewidth=0.5, alpha=0.7, color="black")

# Legend
axs[0].legend(fontsize=16, loc="upper left", frameon=True, edgecolor="black")

# Set the outer frame (spines) to be black and thicker
for spine in axs[0].spines.values():
    spine.set_color("black")
    spine.set_linewidth(2)

# ===============================
# Second Subplot: RPE
# ===============================
axs[1].errorbar(
    x_positions,
    y_rpe,
    yerr=yerr_rpe,
    fmt="o",
    markersize=10,
    capsize=6,
    elinewidth=2,
    capthick=2,
    color="darkred",
    label="Mean ± 0.5x Std Dev",
)

axs[1].set_xlabel("IMU type", fontsize=20, fontweight="bold")
axs[1].set_ylabel("RPE [m]", fontsize=20, fontweight="bold")

# Format X-axis

axs[1].set_xticks(x_positions)
axs[1].tick_params(axis="both", labelsize=20)
axs[1].set_xticklabels(x)  # Set labels
# axs[1].ticklabel_format(axis="x", style="sci", scilimits=(0,0))

# axs[1].xaxis.set_major_locator(mticker.AutoLocator())  # Automatically choose appropriate integer ticks
# axs[1].xaxis.set_minor_locator(mticker.MultipleLocator(0.5))  # Add minor ticks at 0.5 intervals if needed

axs[1].set_xlim(min(x_positions) - 1, max(x_positions) + 1)
# Grid for better readability
axs[1].grid(True, linestyle="--", linewidth=0.5, alpha=0.7, color="black")

# Legend
axs[1].legend(fontsize=16, loc="upper left", frameon=True, edgecolor="black")

# Set the outer frame (spines) to be black and thicker
for spine in axs[1].spines.values():
    spine.set_color("black")
    spine.set_linewidth(2)

# Save the figure as a PDF
fig.savefig(os.path.join(output_dir, "imu_abliation.pdf"), format="pdf", bbox_inches="tight", dpi=600)

plt.tight_layout(pad=2)
plt.show()
