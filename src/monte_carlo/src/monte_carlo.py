#!/usr/bin/env python
import rospy
import tf
import sys
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
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
    _old_odometry = () # contains the odometry used in last iteration of __update_particle_list
    _map = []  # Contains list of cells in map
    _particles = []  # List with particle tuples = (x, y, theta)
    _pose_array = PoseArray()
    _occupancy_grid_msg = OccupancyGrid()
    publisher = None
    _free_space_list = []  # Only free space
    _is_new_odometry = False
    _is_new_laser_data = False
    _first_odom = True
    _real_position = ()
    _position_error = []
    _theta_error = []

    _number_of_particles = 50
    _num_of_measurements = 4 # should be a value of 512 mod(num_measurements) = 0
    _loop_time = 0.3 # Loop time in seconds
    _n_eff = 0 # For resampling
    _sigma = 1.5
    ALFA_1 = 0.01 # rotation
    ALFA_2 = 0.5 # translation
    ALFA_3 = 0.5 # translation
    ALFA_4 = 0.01 # rotation
    _resample_threshold = _number_of_particles / 2


    def __init__(self, testing = False):
        # Initialize mcl node
        rospy.init_node('monte_carlo', anonymous=True)
        if not testing:
            # initializes particles and publisher
            self._initialize_publisher()
            self._initialize_subscribers()

            start_time = time.time()  # start time of loop [seconds]
            self._initialize_particles()
            elapsed_time = time.time() - start_time
            rospy.loginfo("Init particle time:" + str(elapsed_time))
            # Update pose_array
            self._update_pose_array(self._particles)
            # Publish the initial Particles
            self._publish_pose_array(self._pose_array)


    def loop(self):
        # While not mcl is terminated by user
        while not rospy.is_shutdown():
            start_time = time.time()  # start time of loop [seconds]
            if self._is_new_odometry and self._is_new_laser_data:
                # Set flags to false
                self._is_new_laser_data = False
                self._is_new_odometry = False
                midl_odom = self._odometry

                self._particles = self._update_particle_list(self._particles, self._odometry, self._old_odometry, self._laser_point_list,
                                                             self._map)
                self._old_odometry = midl_odom  # set old odom to this odom
                self._pose_array = self._update_pose_array(self._particles)
                self._publish_pose_array(self._pose_array)

                # TEST
                estimated_location_mean = self._calculate_mean_location(self._particles)
                estimated_location_median = self._calculate_median_location(self._particles)
                #rospy.loginfo("Paticle position" + str(self._particles[0]))
                #rospy.loginfo("Real position = " + str(self._real_position))
                error_mean = self.error_measurement(self._real_position, estimated_location_mean)
                error_median = self.error_measurement(self._real_position, estimated_location_median)
                #rospy.loginfo("Error using mean: " + str(error_mean))
                #rospy.loginfo("Error using median:" + str(error_median))
                #rospy.loginfo("\n")

                self._position_error.append(error_mean[0])
                self._theta_error.append(error_mean[1])

                # Loop with constant time.
                elapsed_time = time.time() - start_time
                rospy .loginfo("Loop time = " + str(elapsed_time))

                if elapsed_time > self._loop_time:
                    rospy.loginfo("EXCEED LOOP TIME:" + str(elapsed_time))
                    rospy.loginfo("Number of particles = " + str(len(self._particles)))
                    # break
                elif elapsed_time < self._loop_time:
                    time.sleep(self._loop_time-elapsed_time)


        rospy.loginfo("interrupted!")
        # Saves error to file
        with open('pos_error.txt', 'w+') as f:
            for error in self._position_error:
                f.write("%s\n" % error)

        with open('theta_error.txt', 'w+') as f:
            for error in self._theta_error:
                f.write("%s\n" % error)

    def _update_particle_list(self, old_particles, odometry, old_odometry, laser_points, map):
        """
        calculates new particles based on Monte Carlo Localization.
        :param old_particles: The old list of particles
        :param odometry: The odometry since the last calculation
        :param laser_points: List of all the laser_points
        :param map: the map we want to localize the robot in.
        :return: the new list of particles.
        """
        #Varible that is true while 1 or more particles are inside the map
        all_particles_inside_map = False

        # the new particles
        particle_list = []
        # the old_particles after applying the motion_model
        predicted_particles_list = []
        # the weights of the old_particles after applying the motion_model
        weight_list = []

        # motion and measurement model
        # rospy.loginfo("Predicts new particle positions")
        for particle in old_particles:
            # get new poses after applying the motion_model
            predicted_particle = self.sample_motion_model_odometry(odometry, old_odometry, particle)
            predicted_particles_list.append(predicted_particle)

            # x and y coordinates for particle in the grid map.
            x = int(predicted_particle[0] / self._occupancy_grid_msg.info.resolution)
            y = int(predicted_particle[1] / self._occupancy_grid_msg.info.resolution)

            # if one particle is inside the map all_particles_inside_map will be true
            if self.get_occupancy_value(x, y, map) != -1 and not all_particles_inside_map:
                all_particles_inside_map = True


            # TEST
            # get weights corresponding to the new pose_list
            weight_list.append(self.measurement_model(laser_points, predicted_particle, map))

        self._pose_array = self._update_pose_array(predicted_particles_list)
        self._publish_pose_array(self._pose_array)

        particle_list = predicted_particles_list
        # rospy.loginfo("Particles:" + str(old_particles))
        # rospy.loginfo("Weights:" + str(weight_list))

        if sum(weight_list) == 0:
            start_time = time.time()  # start time of loop [seconds]
            self._initialize_particles()
            elapsed_time = time.time() - start_time
            rospy.loginfo("Init particle loop time after resampling:" + str(elapsed_time))
            return self._particles

        # normalize the weights
        weight_list = self._normalize_weights(weight_list)

        # Before resampling: Check the number of effective particles
        temp = 0
        for weight in weight_list:
           temp = temp + weight**2

        n_eff = 1/temp
        # TODO: This we have to test to see if it resamples enough! Only a tump of rule to use paticles = M/2
        if n_eff < self._resample_threshold:
            # sample the new particles
            #rospy.loginfo("RESAMPLES")
            particle_list = MonteCarlo.low_variance_sampler(predicted_particles_list, weight_list)
        elif not all_particles_inside_map:
            rospy.loginfo("All particles outside map => RESAMPLE")
            self._particles = []
            self._initialize_particles()
            particle_list = self._particles

        #else:
            #rospy.loginfo("Doesnt resample!")

        # return the new set of particles
        return particle_list

    def sample_motion_model_odometry(self, odometry, old_odometry, x_last):
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

        delta_rot_1 = numpy.arctan2(odometry[1]-old_odometry[1], odometry[0]-old_odometry[0]) - old_odometry[2]
        delta_trans = math.sqrt(pow(odometry[0]-old_odometry[0], 2) + pow(odometry[1]-old_odometry[1], 2))
        delta_rot_2 = odometry[2] - old_odometry[2] - delta_rot_1

        delta_rot_1_hat = delta_rot_1 - self.sample(abs(self.ALFA_1 * delta_rot_1 + self.ALFA_2 * delta_trans))
        delta_trans_hat = delta_trans - self.sample(abs(self.ALFA_3 * delta_trans + self.ALFA_4 * (delta_rot_1 + delta_rot_2)))
        delta_rot_2_hat = delta_rot_2 - self.sample(abs(self.ALFA_1 * delta_rot_2 + self.ALFA_2 * delta_trans))

        x = x_last[0] + delta_trans_hat * math.cos(x_last[2] + delta_rot_1)
        y = x_last[1] + delta_trans_hat * math.sin(x_last[2] + delta_rot_1)
        theta = x_last[2] + delta_rot_1_hat + delta_rot_2_hat



        return x, y, theta

    def error_measurement(self, actual_robot_pose, estimated_robot_pose):
       """
       :param actual_robot_pose:
       :param estimated_robot_pose:
       :return: error
       """
       dif_x = (actual_robot_pose[0] - estimated_robot_pose[0])
       dif_y = (actual_robot_pose[1] - estimated_robot_pose[1])
       dif_position = numpy.sqrt(dif_x**2 + dif_y**2)

       estimated_robot_theta = estimated_robot_pose[2] % (numpy.pi*2)
       dif_theta = abs(actual_robot_pose[2] - estimated_robot_theta)
       error = (dif_position, dif_theta)
       # TODO
       #rospy.loginfo("ERROR   sqrt(x,y) = " + str(error[0]) + "     theta error = " + str(error[1]))
       #rospy.loginfo("REAL x =" + str(self._real_position[0]) + "  y =" + str(self._real_position[1]) + "   theta =" + str(self._real_position[2]))
       return error

    def _calculate_mean_location(self, particles):
        x = []
        y = []
        theta = []
        # estimated_location = []

        for particle in particles:
            x.append(particle[0])
            y.append(particle[1])
            theta.append(particle[2])

        x_mean = numpy.mean(x)
        y_mean = numpy.mean(y)
        theta_mean = numpy.mean(theta)

        #rospy.loginfo("ESTIMATED x =" + str(x_mean) + "  y =" + str(y_mean) + "   theta =" + str(theta_mean))

        return (x_mean, y_mean, theta_mean)

    def _calculate_median_location(self, particles):
        x = []
        y = []
        theta = []
        # estimated_location = []

        for particle in particles:
            x.append(particle[0])
            y.append(particle[1])
            theta.append(particle[2])

        x_median = numpy.median(x)
        y_median = numpy.median(y)
        theta_median = numpy.median(theta)


        return (x_median, y_median, theta_median)

    #TEST
    def _normalize_weights(self, weights):
        """
        sums all the weights. divides each weight by weightsum.
        :param weights: a list of all the weights
        :return: a list of the normalized weights.
        """
        sum_weights = sum(weights)
        # return so it doesnt divide by 0
        if sum_weights == 0:
            return weights

        for i in range(0, len(weights)):
            weights[i] = weights[i]/sum_weights

        return weights

    # testing of measurement model
    test = 1
    test3 = True
    #end TEST
    test2 = True
    # testing end

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
        # probability
        # IMPORTANT, weight is set to zero because of logaritmic sum of weights
        weight = 1
        # number of measurements
        num_laser_points = len(laser_points)

        # maximum sensor range [m]
        zmax = 4.3
        # initial value of measurement angle
        theta_k = -numpy.pi / 2
        # degree change for each measurement. 512 points in one scan
        delta_theta = numpy.pi / self._num_of_measurements

        # weight of the elements (change value of z_max to > 0 to include it)
        z_hit = 1
        z_max = 0.1
        z_rand = 0.0
        i = 0

        # initialization of probability
        p_max = 1
        p_rand = 1
        # each measurement laser_points[k] is related to degree from -90 to 90 (given 180 degree range)
        # for loop through all the measurements laser_points
        # laser_points [m]
        # Scanning direction: counterclockwise from Top view
        for laser_point in laser_points[::int(num_laser_points/self._num_of_measurements)]:
            if numpy.isnan(laser_point):
                continue
            # measurement k
            #laser_point = laser_points[k]


            # Real value for laser_point using ray casting
            # (the actual distance to the nearest occupied grid, given predicted_particle and current measurement angle)
            z_t_star = self.ray_casting(predicted_particle, theta_k, map, zmax)

            # if we include z_max; change values of p_hit here.
            if laser_point == zmax:
                p_hit = 1
                p_max = 1
            elif laser_point > zmax:
                p_max = 1
                p_hit = 0
            else:
                # integrate over z_t, integrate.quad gives two values (the integral sum and the error),
                # we are only interested in the integral.
                eta = numpy.divide(1, (scipy.integrate.quad(self.gaussian, 0, zmax, args=(self._sigma, z_t_star))[0]))

                p_hit = eta * self.gaussian(laser_point, self._sigma, z_t_star)
                p_rand = 1/zmax
                p_max = 0

            p = z_hit * p_hit + z_max * p_max + z_rand * p_rand
            weight = p * weight * 1.2

            # the relative direction of the measurement z_t, updated for each iteration
            theta_k = theta_k + delta_theta
            #
            # # TEST to check the last 12 measurements
            # if self.test3:
            #         rospy.loginfo("" + str(i))
            #         rospy.loginfo("p_hit * z_hit: " + str(p_hit*z_hit))
            #         #rospy.loginfo("p_rand * z_rand: " + str(p_rand*z_rand))
            #         #rospy.loginfo("p_max * z_max: " + str(p_max*z_max))
            #         rospy.loginfo("p: " + str(p))
            #         #rospy.loginfo("particle angle: " + str(predicted_particle[2]))
            #         rospy.loginfo("laster_point: " + str(laser_point))
            #         rospy.loginfo("z_t_star: " + str(z_t_star))
            #         rospy.loginfo("eta: " + str(eta))
            #         #rospy.loginfo("theta_k: " + str(theta_k))
            #         rospy.loginfo("Weight: " + str(weight))
            #         rospy.loginfo("\n")
              #      self.test = self.test + 1
                    #if i == 51:
                    #    self.test3 = False
            #TEST e

            i = i + 1

            if numpy.isnan(weight):
                rospy.loginfo("NAN value")
                rospy.loginfo("eta: " + str(eta))
                rospy.loginfo(laser_point)
                weight = 0
                return weight

        #if self.test2 == True:
            #rospy.loginfo("Total weight = " + str(weight))
            #rospy.loginfo("Pose: " +  str(predicted_particle))

            #self.test2 = False

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
            #TODO: change if-sentence when map is 100% (remove "or")
            occupancy_val = self.get_occupancy_value(x_1, y_1, map)
            if map_occupancy_val < occupancy_val or occupancy_val == -1:
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
        # If sum of weights = 0, return the old particle set.
        # TODO: This is the case of kidnapping. Should return new random particle set with in some certain area
        if sum(weights) == 0:
            return predicted_particles

        new_particle_list = []
        r = random.random() * (1 / (len(predicted_particles)))
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
        initial_particle_placement = []  # list with initial cells

        if len(self._free_space_list) == 0:
            # add the element numbers that are free
            for i in range(0, len(self._map)):
                if self._map[i] != 100 and self._map[i] != -1:
                    self._free_space_list.append(i)

        # pick random free cells
        for i in range(0, self._number_of_particles):
            initial_particle_placement.append(random.choice(self._free_space_list))

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
            #self._particles.append((15.880000, 15.240000 , 0))
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
        rospy.Subscriber('/PoseStamped', PoseStamped, self.callback_real_position)




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
        self._is_new_odometry = True

    def callback_laser(self, laserscan):
        """
        callback function used by a ros subscriber. Receives the laser and saves it to a list.
        :param pose: ros massage
        """
        self._laser_point_list = laserscan.ranges

        self._is_new_laser_data = True

    def callback_map(self, occupancy_grid_msg):
        """

        :param occupancy_grid_msg:
        :return:
        """
        self._map = occupancy_grid_msg.data  # Total map
        self._occupancy_grid_msg = occupancy_grid_msg

    def callback_real_position(self, pose_stamped_msg):

        x = -pose_stamped_msg.pose.position.y + 15.880000
        y = pose_stamped_msg.pose.position.x + 15.240000

        quaternion = pose_stamped_msg.pose.orientation

        euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w],
                                                         axes='sxyz')

        self._real_position = (x, y , euler[2] + 1.57079632679)



if __name__ == '__main__':
    mcl = MonteCarlo()
    mcl.loop()
