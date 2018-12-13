from matplotlib import pyplot as plt
import numpy as np

# Read data from text files
pose_error_file = open("pose_error_mean.txt", "r")
theta_error_file = open("theta_error_mean.txt", "r")
pose_error_file_median = open("pose_error_median.txt", "r")
theta_error_file_median = open("theta_error_median.txt", "r")

pose_error = []
for row in pose_error_file:
    pose_error.append(row)

theta_error = []
for row in theta_error_file:
    theta_error.append(row)

pose_error_median = []
for row in pose_error_file_median:
    pose_error_median.append(row)

theta_error_median = []
for row in theta_error_file_median:
    theta_error_median.append(row)

# Create the time array
loop_time = 2
t = np.linspace(0, len(pose_error), loop_time)

plt.figure(1)
plt.plot(t, pose_error)
plt.show()
plt.title("Pose_error_mean")
plt.xlabel("Time [s]")
plt.ylabel("Pose_error [m]")

plt.figure(2)
plt.plot(t, theta_error)
plt.show()
plt.title("Orientation_error_mean")
plt.xlabel("Time [s]")
plt.ylabel("Orientation_error [rad]")

plt.figure(3)
plt.plot(t, pose_error_median)
plt.show()
plt.title("Pose_error_median")
plt.xlabel("Time [s]")
plt.ylabel("Pose_error [m]")

plt.figure(2)
plt.plot(t, theta_error_median)_
plt.show()
plt.title("Orientation_error_median")
plt.xlabel("Time [s]")
plt.ylabel("Orientation_error [rad]")












