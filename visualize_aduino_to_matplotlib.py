import serial
import matplotlib.pyplot as plt
from time import sleep
import numpy as np
import signal
import sys

# Set up serial communication with Arduino
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# Create a plot for filtered coordinates
plt.ion()  # Turn on interactive mode
fig_filtered, ax_filtered = plt.subplots()
line_filtered, = ax_filtered.plot([], [], 'b-')  # 'b-' is for blue line

# Set axis limits for filtered coordinates
ax_filtered.set_xlim(0, 1023)  # Assuming analog read value range (0-1023)
ax_filtered.set_ylim(0, 1023)

# Labels for the axes of filtered coordinates
ax_filtered.set_xlabel('Filtered X Coordinate')
ax_filtered.set_ylabel('Filtered Y Coordinate')
ax_filtered.set_title('Real-Time Filtered Joystick Coordinates')

# Create a plot for raw coordinates
fig_raw, ax_raw = plt.subplots()
scat_raw = ax_raw.scatter([], [], c='r')  # 'r' is for red dots

# Set axis limits for raw coordinates
ax_raw.set_xlim(0, 1023)  # Assuming analog read value range (0-1023)
ax_raw.set_ylim(0, 1023)

# Labels for the axes of raw coordinates
ax_raw.set_xlabel('Raw X Coordinate')
ax_raw.set_ylabel('Raw Y Coordinate')
ax_raw.set_title('Real-Time Raw Joystick Coordinates')

def read_data():
    """Read data from Arduino and return X, Y coordinates and type."""
    line = arduino.readline().decode('utf-8').strip()
    if line:
        print(f"Received: {line}")  # Debugging output to check the received data
        try:
            if line.startswith("Filtered X:"):
                parts = line.split(', ')
                x_str = parts[0].split(': ')[1]
                y_str = parts[1].split(': ')[1]
                x_val = int(x_str)
                y_val = int(y_str)
                return x_val, y_val, 'filtered'
            elif line.startswith("X:"):
                parts = line.split(', ')
                x_str = parts[0].split(': ')[1]
                y_str = parts[1].split(': ')[1]
                x_val = int(x_str)
                y_val = int(y_str)
                return x_val, y_val, 'raw'
            else:
                print(f"Unexpected format: {line}")  # Handle unexpected format
                return None, None, None
        except (ValueError, IndexError) as e:
            print(f"Error parsing data: {e}")  # Error handling
            return None, None, None
    return None, None, None

# Initialize previous values for filtering
prev_x, prev_y = None, None

# Lists to store the filtered coordinates
filtered_x_coords = []
filtered_y_coords = []

# Lists to store the raw coordinates
raw_x_coords = []
raw_y_coords = []

def signal_handler(sig, frame):
    sys.exit(0)

# Register the signal handler for SIGINT
signal.signal(signal.SIGINT, signal_handler)

# Start reading and plotting the joystick coordinates
iteration = 0
max_iterations = 1000  # Set a limit for the number of iterations
while iteration < max_iterations:
    x, y, coord_type = read_data()
    if x is not None and y is not None:
        if coord_type == 'filtered':
            # Append the coordinates to the filtered lists
            filtered_x_coords.append(x)
            filtered_y_coords.append(y)

            # Update the plot with the new filtered coordinates
            line_filtered.set_xdata(list(line_filtered.get_xdata()) + [x])
            line_filtered.set_ydata(list(line_filtered.get_ydata()) + [y])
            ax_filtered.relim()
            ax_filtered.autoscale_view()
            plt.pause(0.01)  # Reduce the pause to 10ms for faster updates
        elif coord_type == 'raw':
            # Append the coordinates to the raw lists
            raw_x_coords.append(x)
            raw_y_coords.append(y)

            # Update the scatter plot with the new raw coordinates
            scat_raw.set_offsets(np.c_[raw_x_coords, raw_y_coords])
            ax_raw.relim()
            ax_raw.autoscale_view()
            plt.pause(0.01)  # Reduce the pause to 10ms for faster updates

    iteration += 1

# Show the filtered coordinates plot
plt.ioff()
plt.show()

# Show the raw coordinates plot after the first plot is finished
plt.figure(fig_raw.number)
plt.show()
