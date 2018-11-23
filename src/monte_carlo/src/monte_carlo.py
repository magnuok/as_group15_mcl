#!/usr/bin/env python
import rospy
import tf
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import random
import math
import numpy
import time
import scipy.integrate


class MonteCarlo:
    """
    Class description
    """

    # TODO: maybe remove these, put them in __init__
    _laser_point_list = []  # List with laserscan. Scanning [pi, 0]. 512 scans
    _odometry = ()  # Contains the new odometry tuppel = (x,y,theta)
    _map = []  # Contains list of cells in map
    _particles = []  # List with particle tuples = (x, y, theta)
    _pose_array = PoseArray()
    _occupancy_grid_msg = OccupancyGrid()
    publisher = None

    _new_odometry = False
    _new_laser_data = False

    def __init__(self):
        # Initialize mcl node
        rospy.init_node('monte_carlo', anonymous=True)
        # initializes particles and publisher
        self._initialize_subscribers()
        self._initialize_publisher()
        self._initialize_particles()

        # Set frame_id in pose_array to be recognized in rviz
        self._pose_array.header.frame_id = "map"

        # Update pose_array
        self._update_pose_array(self._particles)
        # Publish the initial Particles
        self._publish_pose_array(self._pose_array)


    def loop(self):
        # rate = rospy.Rate(1) #Every 1/frequency seconds. Input freq

        # While not mcl is terminated by user
        while not rospy.is_shutdown():
            start_time = time.time()  # start time of loop [seconds]
            if self._new_odometry and self._new_laser_data:
                self._new_laser_data = False
                self._new_odometry = False
                self._particles = self._update_particle_list(self._particles, self._odometry, self._laser_point_list,
                                                             self._map)
                self._pose_array = self._update_pose_array(self._particles)
                self._publish_pose_array()
                # rate.sleep() run loop at exact time in Hz
                elapsed_time = time.time() - start_time
                rospy.loginfo("Loop time:" + str(elapsed_time))

    def _update_particle_list(self, old_particles, odometry, laser_points, map):
        """
        calculates new particles based on Monte Carlo Localization.
        :param old_particles: The old list of particles
        :param odometry: The odometry since the last calculation
        :param laser_points: List of all the laser_points
        :param map: the map we want to localize the robot in.
        :return: the new list of particles.
        """
        # the new particles
        particle_list = []
        # the old_particles after applying the motion_model to them
        predicted_particles_list = []
        # the weights of the old_particles after applying the motion_model to them
        weight_list = []

        # motion and measurement model
        for particle in old_particles:
            # get new poses after applying the motion_model
            predicted_particles_list.append(type(self).sample_motion_model_odometry(odometry, particle))

            # get weights corresponding to the new pose_list
            weight_list.append(type(self).measurement_model(laser_points, particle, map))

        # sample the new particles
        particle_list = type(self).low_variance_sampler(predicted_particles_list, weight_list)

        # return the new set of particles
        return particle_list

    @classmethod
    def sample_motion_model_odometry(cls, odometry, x_last):
        """
        Not sure if it's correct to specify this as a classmethod. Same goes for staticmethod underneath.

        Implementation of the sample_motion_model_odometry as described in
        https://docs.google.com/document/d/1EY1oFbIIb8Ac_3VP40dt_789sVhlVjovoN9eT78llkQ/edit
        :param odometry: a tuple of movement (x, y, theta) where x and y is distance in the plane, and theta is the
        rotation.
        :param x_last: a tuple of old position (x, y, theta) where x and y is position in the plane, and theta is the
        orientation
        :return: returns a tuple which represents the new position (x, y, theta) where x and y is position in the plane,
        and theta is the orientation
        """

        # TODO: find out what these values should be
        # TODO: maybe move these to the top of the class, or the top of the module
        # constants
        ALFA_1 = 0.1;
        ALFA_2 = 0.1;
        ALFA_3 = 0.1;
        ALFA_4 = 0.1;

        delta_rot_1 = numpy.arctan2(odometry[1], odometry[0])
        delta_trans = math.sqrt(odometry[0] ^ 2 + odometry[1] ^ 2)
        delta_rot_2 = odometry[2] - delta_rot_1

        delta_rot_1_hat = delta_rot_1 - cls.sample(abs(ALFA_1 * delta_rot_1 + ALFA_2 * delta_trans))
        delta_trans_hat = delta_trans - cls.sample(abs(ALFA_3 * delta_trans + ALFA_4 * (delta_rot_1 + delta_rot_2)))
        delta_rot_2_hat = delta_rot_2 - cls.sample(abs(ALFA_1 * delta_rot_2 + ALFA_2 * delta_trans))

        x = x_last[0] + delta_trans_hat * math.cos(x_last[2] + delta_rot_1)
        y = x_last[1] + delta_trans_hat * math.sin(x_last[2] + delta_rot_1)
        theta = x_last[2] + delta_rot_1_hat + delta_rot_2_hat

        return x, y, theta

    def measurement_model(self, laser_points, predicted_particle, map):
        """
        Beam range finder model. Function returns the wheoghts of a particle
        :param laser_points: measurement values at time t, list
        :param predicted_particle: state (pose) at time t (x and y - coordinates and theta)
        :param map: grid map, list of values representing occupancy
        :return:
        """

        ### predicted_particle = (x, y, theta)

        sigma = 0.5
        q = 1
        K = len(laser_points)  # number of measurements

        zmax = 5.6  # maximum sensor range [m]
        theta_k = -numpy.pi / 2  # initial value of measurement angle
        delta_theta = numpy.pi / 512  # degree change for each measurement. 512 points in one scan

        # weights of the elements
        z_hit = 0.9
        z_max = 0.1
        # initial values
        p_hit = 1
        p_max = 1

        # each measurement laser_points[k] is related to degree from -90 to 90 (given 180 degree range)
        # for loop through all the measurements laser_points
        # laser_points [m]
        # Scanning direction: counterclockwise from Top view
        for k in K:
            z_t = laser_points[k]  # measurement k
            theta_k = theta_k + delta_theta  # need the relative direction of the measurement z_t
            z_t_star = self.ray_casting(predicted_particle, theta_k, map,
                                        zmax)  # Real value for measurement k using ray casting

            if z_t == z_max:
                p_hit = 0
                p_max = 1
            elif z_t > z_max:
                p_hit = 0
            else:
                # integrate over z_t, integrate.quad gives two values (the integral sum and the error),
                # we are only interested in the integral.
                eta = 1 / (scipy.integrate.quad(self.gaussian, 0, zmax, args=(sigma, z_t_star))[0])
                p_hit = eta * self.gaussian(sigma, z_t_star, z_t)
                p_max = 0

            p = z_hit * p_hit + z_max * p_max
            q = p * q

        return q

    @staticmethod
    def gaussian(sigma, z_t_star, z_t):
        """
        Sub function of measurement_model calculating the Gaussian
        :param z_t_star:
        :param z_t:
        :return:
        """
        return 1 / numpy.sqrt(2 * numpy.pi * sigma ** 2) * numpy.exp(-0.5 * ((z_t_star - z_t) ** 2) / (sigma ** 2))

    def ray_casting(self, predicted_particle, theta_k, map, zmax):
        """
        Sub function of measurement_model using current state (pose of robot) and map to calculate true value of a measurement z_t
        :param theta_k:
        :param map:
        :param zmax:
        :return: true distance in meter to obstacle/occupied grid
        """
        x_0 = predicted_particle[0]
        y_0 = predicted_particle[1]
        theta = predicted_particle[
                    2] + theta_k  # Need to know the angel of both robot and measurement to know in which direction to ray cast.
        distance = zmax  # initialize shortest distance to occupied grid as maximal value
        m_size = len(map)  # map size
        max_map_value = zmax  # the maximum value in the map given the particle
        map_occupancy_val = 80  # value to identify occupancy
        if x_0 < max_map_value / 2 and y_0 < max_map_value / 2:
            x_max = m_size - x_0
            y_max = m_size - y_0
        elif x_0 > max_map_value / 2 and y_0 > max_map_value / 2:
            x_max = x_0
            y_max = y_0
        elif x_0 < max_map_value / 2 and y_0 > max_map_value / 2:
            x_max = m_size - x_0
            y_max = y_0
        else:
            x_max = x_0
            y_max = m_size - y_0

        max_map_value = numpy.sqrt(y_max ** 2 + x_max ** 2)

        # Uses max_map_value to find end points to use in the bresenham algorithm
        # numpy.cos() and numpy.sin() uses radians.
        x_end = x_0 + round(max_map_value * numpy.cos(theta))
        y_end = y_0 + round(max_map_value * numpy.sin(theta))

        # Gets grids between start and end points
        grids = self.bresenham(x_0, y_0, x_end, y_end)

        # find the maximum x and y value the first occupied grid may have.

        # Start going through the grids provided by Bresenham and when we find a grid with higher occupance probability
        # value we calculate and return the distance to this value
        # We assume that Bresnham returns an array of grids where the distance from x_0 and y_0 increases. TEST THIS
        for grid in grids:
            x_1 = grid[0]
            y_1 = grid[1]
            # occupancy value in map of the given grid
            occupancy_val = self.get_grid_position(x_1, y_1, map)
            if map_occupancy_val < occupancy_val:  # TODO krokodille
                distance = numpy.sqrt((x_1 - x_0) ** 2 + (y_1 - y_0) ** 2)
                break
        # return distance in meter
        return distance * self._occupancy_grid_msg.info.resolution

    @staticmethod
    def get_grid_position(self, row, column, map):
        # TODO: check that this is right with col and row
        width = self._occupancy_grid_msg.info.width
        return map[row * width + column]

    @staticmethod
    def sample(standard_deviation):
        """
        :param standard_deviation: standard deviation.
        :return: A number chosen randomly from the normal distribution with center in 0 and standard deviation =
        standard_deviation
        """
        return numpy.random.normal(loc=0, scale=standard_deviation, size=None)

    @staticmethod
    def bresenham(x_0, y_0, x_end, y_end):
        """
        Sub function of ray_casting used for calculations of true value of z_t
        :param y_0:
        :param x_end:
        :param y_end:
        :return:
        """
        # Bresenham returns all the x-y coordinates of grids between (x_0, y_0) and (x_end, y_end)
        # The end points will post likely go outside of our grid map (keep in mind)
        dx = x_end - x_0
        dy = y_end - y_0

        # if the thetha is over 45 degrees we rotate the line (mirror the line)
        is_steep = abs(dx) < abs(dy)

        if is_steep:
            x_0, y_0 = y_0, x_0
            x_end, y_end = y_end, x_end

        # checks if dx is negative, if so rotates the line again => we always look at a line going in positive x direction
        # if dx is negative we change the direction of the "arrow"
        swapped = False
        if x_0 > x_end:
            x_0, x_end = x_end, x_0
            y_0, y_end = y_end, y_0
            swapped = True

        dx = x_end - x_0
        dy = y_end - y_0

        # initial value of error
        error = dx / 2

        if y_0 < y_end:
            y_step = 1
        else:
            y_step = -1

        # initial y-value to start point
        y = y_0
        # empty array of grid coordinates
        grids = []

        # iterates over x-coordinates (may be y-coordinates, if is_steep = true)
        # iterates over each x between x_0 and x_end
        # The error first get subtracted
        for x in range(x_0, x_end + 1):
            if is_steep:
                coord = (y, x)
            else:
                coord = (x, y)
            grids.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx

        # reverse back list if they were reversed.
        if swapped:
            grids.reverse()

        return grids

    @staticmethod
    def low_variance_sampler(predicted_particles, weights):
        """
        :param predicted_particles:
        :param weights:
        :return:
        """
        # TODO: maybe implement exception
        new_particle_list = []
        r = random.random() * (1. / (len(predicted_particles)))
        c = weights[0]
        i = 0

        for m in range(1, len(predicted_particles) + 1):
            u = r + (m - 1) * (1. / (len(predicted_particles)))
            while u > c:
                i += 1
                c += weights[i]
            new_particle_list.append(predicted_particles[i])
        return new_particle_list

    def _update_pose_array(self, _particles):
        for particle in self._particles:
            pose = self._create_pose(particle)
            self._pose_array.poses.append(pose)  # Add to pose_array
        pass

    def _initialize_particles(self):
        """
        Method that initialize the particles according to map
        :param:
        :return:
        """

        # Dimensions
        resolution = self._occupancy_grid_msg.info.resolution
        width = self._occupancy_grid_msg.info.width

        # The cell arrays:
        free_space_list = []  # Only free space
        initial_particle_placement = []  # list with initial cells

        # add the element numbers that are free
        for i in range(0, len(self._map)):
            if self._map[i] != 100 and self._map[i] != -1:
                free_space_list.append(i)

        # pick random free cells
        number_of_particles = 100
        for i in range(0, number_of_particles):
            initial_particle_placement.append(random.choice(free_space_list))

        # Find the corresponding row and col number of each particle
        # Add the particle tuple (x, y, theta) to initial_particle_placement
        for index in range(0, len(initial_particle_placement)):
            index_map_array = initial_particle_placement[index]
            row = 0

            while index_map_array > width:
                row = row + 1
                index_map_array = index_map_array - width
            col = index_map_array

            # Calculate distance
            particle_width = row * resolution
            particle_height = col * resolution

            # Adds all particles to list SHOULD CHANGE NAMES HERE TO GET WIDTH ON X AND HEIGHT ON Y
            self._particles.append(
                (particle_height, particle_width, random.uniform(0, 2 * math.pi)))

    def _initialize_publisher(self):
        # initialize the publisher object
        self.publisher = rospy.Publisher('/PoseArray', PoseArray, queue_size=10)

    def _initialize_subscribers(self):
        rospy.Subscriber("/map", OccupancyGrid, self.callback_map)
        rospy.Subscriber("/RosAria/pose", Odometry, self.callback_odometry)
        rospy.Subscriber("/scan", LaserScan, self.callback_laser)


    def _publish_pose_array(self, _pose_array):
        """
        :return:
        """

        # publishes the particles
        self.publisher.publish(_pose_array)

    def _create_pose(self, particle):
        """
        Retrives a particle and transforms to Pose object. Used to publish particles to rviz through /PoseArray topic
        :param particle: particle tuppel =(x, y, theta)
        :return pose: Pose object
        """
        pose = Pose()
        # Add x,y,z position to pose
        pose.position.x = particle[0]
        pose.position.y = particle[1]
        pose.position.z = 0
        # add quaternion to pose
        quaternion = tf.transformations.quaternion_from_euler(0, 0, particle[2], 'sxyz')
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def callback_odometry(self, odom_msg):
        """
        callback function used by a ros subscriber. Receives the odometry and saves it to a list.
        :param pose: ros pose
        """
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        # Transform quaternion to euler
        quaternion = odom_msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self._odometry = (x, y, euler[2])

    def callback_laser(self, laserscan):
        """
        callback function used by a ros subscriber. Receives the laser and saves it to a list.
        :param pose: ros massage
        """
        self._laser_point_list = laserscan.ranges

    def callback_map(self, occupancy_grid_msg):
        """

        :param occupancy_grid_msg:
        :return:
        """
        self._map = occupancy_grid_msg.data  # Total map
        self._occupancy_grid_msg = occupancy_grid_msg


if __name__ == '__main__':
    mcl = MonteCarlo()
    #mcl.loop()
    rospy.spin()