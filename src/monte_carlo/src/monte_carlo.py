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
    _laser_point_list = []  # List with laserscan. Scanning [pi, 0]. 512 scanss
    _odometry = ()  # Contains the new odometry tuppel = (x,y,theta)
    _old_odometry = (0, 0, 0) # contains the odometry used in last iteration of __update_particle_list
    _map = []  # Contains list of cells in map
    _particles = []  # List with particle tuples = (x, y, theta)
    _pose_array = PoseArray()
    _occupancy_grid_msg = OccupancyGrid()
    publisher = None

    _number_of_particles = 1

    _new_odometry = False
    _new_laser_data = False
    _first_odom = True

    def __init__(self, testing = False):
        # Initialize mcl node
        rospy.init_node('monte_carlo', anonymous=True)
        if not testing:
            # initializes particles and publisher
            self._initialize_publisher()
            self._initialize_subscribers()
            self._initialize_particles()

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
                # Set flags to false
                self._new_laser_data = False
                self._new_odometry = False

                odometry = tuple(numpy.subtract(self._odometry, self._old_odometry))
                self._particles = self._update_particle_list(self._particles, odometry, self._laser_point_list,
                                                             self._map)
                self._old_odometry = self._odometry  # set old odom to this odom
                self._pose_array = self._update_pose_array(self._particles)
                self._publish_pose_array(self._pose_array)
                # rate.sleep() run loop at exact time in Hz
                elapsed_time = time.time() - start_time
                #rospy.loginfo("Loop time:" + str(elapsed_time))

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
        # the old_particles after applying the motion_model
        predicted_particles_list = []
        # the weights of the old_particles after applying the motion_model
        weight_list = []

        # motion and measurement model
        for particle in old_particles:
            # get new poses after applying the motion_model
            # TEST
            predicted_particle = MonteCarlo.sample_motion_model_odometry(odometry, particle)
            predicted_particles_list.append(predicted_particle)
            # end TEST
            # get weights corresponding to the new pose_list

            # midl
            predicted_particle = particle
            # end midl

            weight_list.append(self.measurement_model(laser_points, predicted_particle, map))

        # test
        particle_list = old_particles
        # end test
        #rospy.loginfo("Particles:" + str(old_particles))
        #rospy.loginfo("Weights:" + str(weight_list))
        # sample the new particles
        #particle_list = MonteCarlo.low_variance_sampler(predicted_particles_list, weight_list)

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

        ALFA_1 = 0.001;
        ALFA_2 = 0.1;
        ALFA_3 = 0.1;
        ALFA_4 = 0.001;

        delta_rot_1 = numpy.arctan2(odometry[1], odometry[0])
        delta_trans = math.sqrt(pow(odometry[0], 2) + pow(odometry[1], 2))
        delta_rot_2 = odometry[2] - delta_rot_1

        delta_rot_1_hat = delta_rot_1 - cls.sample(abs(ALFA_1 * delta_rot_1 + ALFA_2 * delta_trans))
        delta_trans_hat = delta_trans - cls.sample(abs(ALFA_3 * delta_trans + ALFA_4 * (delta_rot_1 + delta_rot_2)))
        delta_rot_2_hat = delta_rot_2 - cls.sample(abs(ALFA_1 * delta_rot_2 + ALFA_2 * delta_trans))

        x = x_last[0] + delta_trans_hat * math.cos(x_last[2] + delta_rot_1)
        y = x_last[1] + delta_trans_hat * math.sin(x_last[2] + delta_rot_1)
        theta = x_last[2] + delta_rot_1_hat + delta_rot_2_hat

        return x, y, theta

    test = 1
    test2 = True

    def measurement_model(self, laser_points, predicted_particle, map):
        """
        Beam range finder model. Function returns the wheoghts of a particle
        :param laser_points: measurement values at time t, list
        :param predicted_particle: state (pose) at time t (x and y - coordinates and theta)
        :param map: grid map, list of values representing occupancy
        :return:
        """

        ### predicted_particle = (x, y, theta)

        # standard deviation ( = variance**2 = 0.2 for our SD)
        sigma = 0.4472135955
        # probability
        weight = 1
        # number of measurements
        num_laser_points = len(laser_points)

        # maximum sensor range [m]
        zmax = 5.6
        # initial value of measurement angle
        theta_k = -numpy.pi / 2
        # degree change for each measurement. 512 points in one scan
        delta_theta = numpy.pi / num_laser_points

        # weight of the elements (change value of z_max to > 0 to include it)
        z_hit = 1
        z_max = 0

        # initialization of probability
        p_max = 1

        # each measurement laser_points[k] is related to degree from -90 to 90 (given 180 degree range)
        # for loop through all the measurements laser_points
        # laser_points [m]
        # Scanning direction: counterclockwise from Top view
        for k in range(0, num_laser_points):
            # measurement k
            laser_point = laser_points[k]


            # Real value for laser_point using ray casting
            # (the actual distance to the nearest occupied grid, given predicted_particle and current measurement angle)
            z_t_star = self.ray_casting(predicted_particle, theta_k, map, zmax)

            if laser_point == zmax:
                p_hit = 0
                p_max = 1
            elif laser_point > zmax:
                p_hit = 0
            else:
                # integrate over z_t, integrate.quad gives two values (the integral sum and the error),
                # we are only interested in the integral.
                eta = numpy.divide(1, (scipy.integrate.quad(self.gaussian, 0, zmax, args=(sigma, z_t_star))[0]))

                p_hit = eta * self.gaussian(laser_point, sigma, z_t_star)

                # TEST
                if (self.test < 10):
                    rospy.loginfo("p_hit: " + str(p_hit))
                    rospy.loginfo("predicted particle: " + str(predicted_particle[0]))
                    rospy.loginfo("laster_point: " + str(laser_point))
                    rospy.loginfo("z_t_star: " + str(z_t_star))
                    rospy.loginfo("eta: " + str(eta))
                    rospy.loginfo("theta_k: " + str(theta_k))
                    rospy.loginfo("Weight: " + str(weight))
                    rospy.loginfo("\n")
                    self.test = self.test + 1
                #TEST end

                p_max = 0

            p = z_hit * p_hit + z_max * p_max
            weight = p * weight

            # the relative direction of the measurement z_t, updated for each iteration
            theta_k = theta_k + delta_theta

        if self.test2 == True:
            rospy.loginfo("Total weight = " + str(weight))
            self.test2 = False

        return weight

    @staticmethod
    def gaussian(z_t, sigma, z_t_star):
        """
        Sub function of measurement_model calculating the Gaussian
        :param z_t_star:
        :param z_t:
        :return:
        """
        return (1 / numpy.sqrt(2 * numpy.pi * sigma ** 2)) * numpy.exp(-0.5 * ((z_t_star - z_t) ** 2) / (sigma ** 2))

    def ray_casting(self, predicted_particle, theta_k, map, zmax):
        """
        Sub function of measurement_model using current state (pose of robot) and map to calculate true value of a measurement z_t
        :param theta_k: the direction of the laser point
        :param map: the map
        :param zmax: maximum distance of the laser range finder
        :return: true distance in meter to obstacle/occupied grid
        """
        # grid position in x- and y-direction of the given predicted particle
        x_0 = int(predicted_particle[0] / self._occupancy_grid_msg.info.resolution)
        y_0 = int(predicted_particle[1] / self._occupancy_grid_msg.info.resolution)

        # Direction to ray cast.
        theta = predicted_particle[2] + theta_k

        # initialize shortest distance to occupied grid as maximal value
        distance = zmax / self._occupancy_grid_msg.info.resolution

        # value to identify occupancy
        map_occupancy_val = 80

        # # map size
        # m_size = len(map)
        #
        # # the maximum value in the map given the particle
        # max_map_value = zmax
        #
        #
        # if x_0 < max_map_value / 2 and y_0 < max_map_value / 2:
        #     x_max = m_size - x_0
        #     y_max = m_size - y_0
        # elif x_0 > max_map_value / 2 and y_0 > max_map_value / 2:
        #     x_max = x_0
        #     y_max = y_0
        # elif x_0 < max_map_value / 2 and y_0 > max_map_value / 2:
        #     x_max = m_size - x_0
        #     y_max = y_0
        # else:
        #     x_max = x_0
        #     y_max = m_size - y_0
        #
        # max_map_value = numpy.sqrt(y_max ** 2 + x_max ** 2)

        # Uses max_map_value to find end points to use in the bresenham algorithm
        # numpy.cos() and numpy.sin() uses radians.
        x_end = x_0 + int(distance * numpy.cos(theta))
        y_end = y_0 + int(distance * numpy.sin(theta))

        # Gets grids between start and end points
        grids = self.bresenham(x_0, y_0, x_end, y_end)

        # find the maximum x and y value the first occupied grid may have.

        # Start going through the grids provided by Bresenham and when we find a grid with higher occupancy probability
        # value we calculate and return the distance to this value
        # We assume that Bresnham returns an array of grids where the distance from x_0 and y_0 increases. TEST THIS
        for grid in grids:
            x_1 = grid[0]
            y_1 = grid[1]
            # occupancy value in map of the given grid
            occupancy_val = self.get_occupancy_value(x_1, y_1, map)
            if map_occupancy_val < occupancy_val:
                distance = numpy.sqrt((x_1 - x_0) ** 2 + (y_1 - y_0) ** 2)
                break
        # return distance in meter
        return distance * self._occupancy_grid_msg.info.resolution

    def get_occupancy_value(self, x, y, map):
        """
        :param row:
        :param column:
        :param map:
        :return: occupancy value given x and y coordinate in the grid
        """
        width = self._occupancy_grid_msg.info.width
        height = self._occupancy_grid_msg.info.height
        if y > height or x > width - 1 or x < 1 or y < 0:
            return -1
        else:
            return map[(y - 1) * width + x-1]

    @staticmethod
    def sample(standard_deviation):
        """
        :param standard_deviation: standard deviation.
        :return: A number chosen randomly from the normal distribution with center in 0 and standard deviation =
        standard_deviation
        """
        if standard_deviation == 0:
            return 0
        return numpy.random.normal(loc=0, scale=standard_deviation, size=None)

    @staticmethod
    def bresenham(x_0, y_0, x_end, y_end):
        """
        Sub function of ray_casting used for calculations of true value of z_t
        :param y_0:
        :param x_end:
        :param y_end:
        :return: list of all grid cells between (x_0, y_0) and (x_end, y_end)
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

    def _update_pose_array(self, particles):
        self._pose_array = PoseArray()
        self._pose_array.header.frame_id = "map"
        for particle in particles:
            pose = self._create_pose(particle)
            self._pose_array.poses.append(pose)  # Add to pose_array
        return self._pose_array

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
        for i in range(0, self._number_of_particles):
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
            # TODO: check if its correct
            self._particles.append((particle_height, particle_width, random.uniform(0, 2 * math.pi)))

    def _initialize_publisher(self):
        # initialize the publisher object
        self.publisher = rospy.Publisher('/PoseArray', PoseArray, queue_size=10)
        # Set frame_id in pose_array to be recognized in rviz
        self._pose_array.header.frame_id = "map" # Not be here??

    def _initialize_subscribers(self):
        rospy.Subscriber("/map", OccupancyGrid, self.callback_map)
        # Wait for map to get loaded into self._map
        while len(self._map) == 0:
            pass

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
        # If first odom. Set _old_odometry aswell
        if self._first_odom:
            self._old_odometry = (x, y, euler[2])
            self._first_odom = False

        self._odometry = (x, y, euler[2])
        self._new_odometry = True

    def callback_laser(self, laserscan):
        """
        callback function used by a ros subscriber. Receives the laser and saves it to a list.
        :param pose: ros massage
        """
        self._laser_point_list = laserscan.ranges

        self._new_laser_data = True

    def callback_map(self, occupancy_grid_msg):
        """

        :param occupancy_grid_msg:
        :return:
        """
        self._map = occupancy_grid_msg.data  # Total map
        self._occupancy_grid_msg = occupancy_grid_msg


if __name__ == '__main__':
    mcl = MonteCarlo()
    mcl.loop()
    #rospy.spin()
