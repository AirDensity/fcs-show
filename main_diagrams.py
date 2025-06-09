import matplotlib.pyplot as plt
import sys
import numpy as np  # For mean and median calculations

# Boolean flags to control plotting of median and mean
plot_median = True
plot_mean = True

# Format: _,_,_;_,_,_;...\n

acceleration = []
velocity = []
position = []
angular_acc = []
angular_rates = []
orientation = []


def append(index, value):
    if index == 0:
        acceleration.append(value)
    elif index == 1:
        velocity.append(value)
    elif index == 2:
        position.append(value)
    elif index == 3:
        angular_acc.append(value)
    elif index == 4:
        angular_rates.append(value)
    elif index == 5:
        orientation.append(value)


def plot(value, title):
    x = list(range(0, len(value)))
    v_zero = [float(specific[0]) for specific in value]
    v_one = [float(specific[1]) for specific in value]
    v_two = [float(specific[2]) for specific in value]

    # Calculate the mean and median if enabled
    if plot_mean:
        mean_x = np.mean(v_zero)
        mean_y = np.mean(v_one)
        mean_z = np.mean(v_two)

    if plot_median:
        median_x = np.median(v_zero)
        median_y = np.median(v_one)
        median_z = np.median(v_two)

    fig, axs = plt.subplots(figsize=(15, 15), layout='constrained')
    axs.set_title(title)

    # Plot the original data with more distinct colors
    axs.plot(x, v_zero, label="x", color='#1f77b4')  # A strong blue
    axs.plot(x, v_one, label="y", color='#ff7f0e')  # A vibrant orange
    axs.plot(x, v_two, label="z", color='#2ca02c')  # A solid green

    # Plot mean if enabled with distinguishable, soft but noticeable colors
    if plot_mean:
        axs.axhline(mean_x, color='#d62728', linestyle='--', label="Mean x")  # A bold red
        axs.axhline(mean_y, color='#9467bd', linestyle='--', label="Mean y")  # A subtle purple
        axs.axhline(mean_z, color='#8c564b', linestyle='--', label="Mean z")  # A muted brown

    # Plot median if enabled with clear, but not overpowering colors
    if plot_median:
        axs.axhline(median_x, color='#e377c2', linestyle=':', label="Median x")  # A light pink
        axs.axhline(median_y, color='#7f7f7f', linestyle=':', label="Median y")  # A neutral gray
        axs.axhline(median_z, color='#bcbd22', linestyle=':', label="Median z")  # A warm olive green

    fig.legend(loc='upper right')
    # fig.show()
    fig.savefig(title + ".png")


# Open the specified file if existent
try:
    file = open("Data.txt", 'r')
except FileNotFoundError:
    print("File not found")
    sys.exit()

# Read each line and parse the values to the corresponding list
for line in file.readlines():
    line = line.replace("\n", "")
    content = line.split(";")
    for i, c in enumerate(content):
        vector = c.split(",")
        append(i, vector)

# Plot all lists
plot(acceleration, "Acceleration")
plot(velocity, "Velocity")
plot(position, "Position")
plot(angular_acc, "Angular acceleration")
plot(angular_rates, "Angular rates")
plot(orientation, "Orientation")
