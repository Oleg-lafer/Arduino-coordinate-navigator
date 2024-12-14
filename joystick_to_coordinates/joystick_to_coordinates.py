import serial
import matplotlib.pyplot as plt
from time import sleep
import numpy as np
import signal
import sys

# Set up serial communication with Arduino
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# Create a plot
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()
line, = plt.plot([], [], 'b-')  # 'b-' is for blue line

# Set axis limits
ax.set_xlim(0, 1023)  # Assuming analog read value range (0-1023)
ax.set_ylim(0, 1023)

# Labels for the axes
ax.set_xlabel('Joystick X Coordinate')
ax.set_ylabel('Joystick Y Coordinate')
ax.set_title('Real-Time Joystick Coordinates')

def read_data():
    """Read data from Arduino and return X, Y coordinates."""
    line = arduino.readline().decode('utf-8').strip()
    if line:
        print(f"Received: {line}")  # Debugging output to check the received data
        try:
            # Split the data string from Arduino (X: <value>, Y: <value>)
            parts = line.split(', ')
            if len(parts) == 2:
                x_str = parts[0].split(': ')[1]
                y_str = parts[1].split(': ')[1]
                x_val = int(x_str)
                y_val = int(y_str)
                return x_val, y_val
            else:
                print(f"Unexpected format: {line}")  # Handle unexpected format
                return None, None
        except (ValueError, IndexError) as e:
            print(f"Error parsing data: {e}")  # Error handling
            return None, None
    return None, None

# Low-pass filter function
def low_pass_filter(new_value, prev_value, alpha=0.15):
    return alpha * new_value + (1 - alpha) * prev_value

# Initialize previous values for filtering
prev_x, prev_y = None, None

# Lists to store the filtered coordinates
x_coords = []
y_coords = []

def signal_handler(sig, frame):
    sys.exit(0)

# Register the signal handler for SIGINT
signal.signal(signal.SIGINT, signal_handler)

# Open a file to write the coordinates
with open('coordinates.txt', 'a') as file:
    # Start reading and plotting the joystick coordinates
    iteration = 0
    max_iterations = 1000  # Set a limit for the number of iterations
    while iteration < max_iterations:
        x, y = read_data()
        if x is not None and y is not None:
            if prev_x is None or prev_y is None:
                prev_x, prev_y = x, y
            else:
                x = low_pass_filter(x, prev_x)
                y = low_pass_filter(y, prev_y)
                prev_x, prev_y = x, y

            # Convert filtered coordinates to integers
            x_int = int(x)
            y_int = int(y)

            # Append the coordinates to the lists
            x_coords.append(x_int)
            y_coords.append(y_int)

            # Write the filtered coordinates to the file
            file.write(f"Filtered Coordinates: X={x_int}, Y={y_int}\n")

            # Update the plot with the new coordinates
            line.set_xdata(list(line.get_xdata()) + [x_int])
            line.set_ydata(list(line.get_ydata()) + [y_int])
            plt.pause(0.01)  # Reduce the pause to 50ms for faster updates

        iteration += 1
