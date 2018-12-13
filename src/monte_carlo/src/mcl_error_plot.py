from matplotlib import pyplot as plt
import numpy as np

# Read data from text files
pose_error_file = open("pose_error.txt", "r")
theta_error_file = open("theta_error.txt", "r")

pose_error = []
for row in pose_error_file:
    pose_error.append(row)

theta_error = []
for row in theta_error_file:
    theta_error.append(row)

# Create the time array
loop_time = 2
t = np.linspace(0, len(pose_error), loop_time)

plt.figure(1)
plt.plot(t, pose_error)
plt.show()
plt.title("Pose_error")
plt.xlabel("Time [s]")
plt.ylabel("Pose_error [m]")

plt.figure(2)
plt.plot(t, theta_error)
plt.show()
plt.title("Orientation_error")
plt.xlabel("Time [s]")
plt.ylabel("Orientation_error [rad]")














