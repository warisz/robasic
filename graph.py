import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Initialize lists to store x and y values
x_data = []
y_data = []

# Create a figure and axis for plotting
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)  # Unpack the first element from the list returned by plot()
ax.set_xlim(0, 5000)  # Set x-axis limits (adjust as needed)
ax.set_ylim(0, 8)  # Set y-axis limits (adjust as needed)
ax.set_xlabel('Time (ms)')
ax.set_ylabel('Altitude (m)')
ax.set_title('PID Controller Settling Response')

def init():
    """Initialize the background of the plot."""
    line.set_data([], [])
    return line,

def update(frame):
    """Update the plot with new data."""
    try:
        # Read the latest data from the file
        with open('/Users/warisz/Code/robasic/data.txt', 'r') as f:
            lines = f.readlines()
            y_data.clear()  # Clear previous data
            for value in lines:
                y_data.append(float(value.strip()))

        # Update x_data based on the current length of y_data
        x_data[:] = range(len(y_data))

        # Update line data
        line.set_data(x_data, y_data)  # Update the line with new data

        return line,  # Return as a tuple

    except Exception as e:
        print(f"Error reading file: {e}")
        return line,  # Return as a tuple even on error

# Create an animation that updates every second (1000 milliseconds)
ani = animation.FuncAnimation(fig, update, init_func=init, interval=1000)

plt.show()