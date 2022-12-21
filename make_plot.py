import matplotlib.pyplot as plt
import sys
import math

# Get the name of the input file from the command line arguments
input_file = sys.argv[1]

# Open the input file and read the data
with open(input_file, 'r') as f:
    lines = f.readlines()
    
# Initialize empty lists for x and y data
x_vel_data = []
y_vel_data = []
theta_vel_data = []

x_pos_data = []
y_pos_data = []
theta_pos_data = []

x_pose_data = []
y_pose_data = []
theta_pose_data = []


# Iterate through the lines and extract the x and y values
for line in lines:
    id, x, y, theta = line.split()
    if id == "vel":
        x_vel_data.append(float(x))
        y_vel_data.append(float(y))
        theta_vel_data.append(float(theta))
    if id == "pos":
        x_pos_data.append(float(x))
        y_pos_data.append(float(y))
        theta_pos_data.append(float(theta))
    if id == "pose":
        x_pose_data.append(float(x))
        y_pose_data.append(float(y))
        theta_pose_data.append(float(theta))

# Plot trajectory
plt.figure(1)
plt.title('Trajectory from odom square right')
plt.xlabel("X-values")
plt.ylabel("Y-values")
plt.scatter(x_vel_data, y_vel_data, label='velocity')
plt.scatter(x_pos_data, y_pos_data, label='position')
plt.plot(x_pose_data, y_pose_data, color='red', label='true pose')
plt.grid()
plt.legend()

##### Plot errors
n = list(range(0, len(x_pose_data)))

# X
x_pos_error = [x_pose - x_pos for x_pose, x_pos in zip(x_pose_data, x_pos_data)]
x_vel_error = [x_pose - x_vel for x_pose, x_vel in zip(x_pose_data, x_vel_data)]

plt.figure(2)
plt.title('X-error')
plt.ylabel("Error")
plt.xlabel("N")
plt.scatter(n, x_vel_error, label='velocity')
plt.scatter(n, x_pos_error, label='position')
plt.plot(n, [0] * len(x_pose_data), color='red', label='true pose')
plt.grid()
plt.legend()

# Y
y_pos_error = [y_pose - y_pos for y_pose, y_pos in zip(y_pose_data, y_pos_data)]
y_vel_error = [y_pose - y_vel for y_pose, y_vel in zip(y_pose_data, y_vel_data)]

plt.figure(3)
plt.title('Y-error')
plt.ylabel("Error")
plt.xlabel("N")
plt.scatter(n, y_vel_error, label='velocity')
plt.scatter(n, y_pos_error, label='position')
plt.plot(n, [0] * len(y_pose_data), color='red', label='true pose')
plt.grid()
plt.legend()

# theta
theta_pos_error = [(theta_pos - theta_pose) % (2 * math.pi) for theta_pose, theta_pos in zip(theta_pose_data, theta_pos_data)]
theta_vel_error = [(theta_vel - theta_pose) % (2 * math.pi)  for theta_pose, theta_vel in zip(theta_pose_data, theta_vel_data)]

plt.figure(4)
plt.title('Theta-error')
plt.ylabel("Error")
plt.xlabel("N")
plt.scatter(n, theta_vel_error, label='velocity')
plt.scatter(n, theta_pos_error, label='position')
plt.plot(n, [0] * len(theta_pose_data), color='red', label='true pose')
plt.grid()
plt.legend()

# Show the plot
plt.show()
