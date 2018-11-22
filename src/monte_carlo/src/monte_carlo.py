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
import time  # TEST


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
        self._initialize_particles()
        self._initialize_publisher()
        self._initialize_subscribers()

        # Set frame_id in pose_array to be recognized in rviz
        self._pose_array.header.frame_id = "map"

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
            weight_list.append(type(self).measurement_model(laser_points, predicted_particles_list, map))

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

    @classmethod
    def measurement_model(cls, laser_points, particles, map):
        """

        :param laser_points:
        :param particles:
        :param map:
        :return:
        """
        pass

    @staticmethod
    def sample(standard_deviation):
        """
        :param standard_deviation: standard deviation.
        :return: A number chosen randomly from the normal distribution with center in 0 and standard deviation =
        standard_deviation
        """
        return numpy.random.normal(loc=0, scale=standard_deviation, size=None)

    @staticmethod
    def low_variance_sampler(particles, weights):
        """
        :param particles:
        :param weights:
        :return:
        """
        new_particle_list = []
        r = random.random() * (1. / (len(particles)))
        c = weights[0]
        i = 0

        for m in range(1, len(particles) + 1):
            u = r + (m - 1) * (1. / (len(particles)))
            while u > c:
                i += 1
                c += weights[i]
            new_particle_list.append(particles[i])
        return new_particle_list

    def _update_pose_array(self, _particles):
        for particle in self._particles:
            pose = self._create_pose(particle)
            self._pose_array.poses.append(pose)  # Add to pose_array
        pass

    def _initialize_particles(self):
        """

        :param:
        :return:
        """

        # Dimensions
        resolution = self._occupancy_grid_msg.info.resolution
        width = self._occupancy_grid_msg.info.width

        # rospy.loginfo("Height [m]:" + str(height))
        # rospy.loginfo("Width [m]:" + str(width))
        # #Origin:
        # x_origin = occupancy_grid_msg.info.origin.position.x
        # y_origin = occupancy_grid_msg.info.origin.position.y
        # z_origin = occupancy_grid_msg.info.origin.position.z
        # rospy.loginfo("x_origin:" + str(x_origin))
        # rospy.loginfo("y_origin:" + str(y_origin))
        # rospy.loginfo("z_origin:" + str(z_origin))

        # The cell arrays:

        # rospy.loginfo("_map" + str(self._map))
        free_space_list = []  # Only free space
        initial_particle_placement = []  # list with initial cells

        # add the element numbers that are free
        for i in range(0, len(self._map)):
            if self._map[i] != 100 and self._map[i] != -1:
                free_space_list.append(i)
        rospy.loginfo("Free space list: " + str(free_space_list))

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
        rospy.Subscriber("/map", OccupancyGrid, mcl.callback_map)
        rospy.Subscriber("/RosAria/pose", Odometry, mcl.callback_odometry)
        rospy.Subscriber("/scan", LaserScan, mcl.callback_laser)

    def _publish_pose_array(self):
        """
        :return:
        """

        # publishes the particles
        self.publisher.publish(self._pose_array)

    def _create_pose(self, particle):
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

        self._new_odometry = True

        # rospy.loginfo(self._odometry)

        # self._odometry = pose

        # rospy.loginfo("Received Odom pose:")
        # rospy.loginfo("Timestamp" + str(self._odometry.header.stamp))
        # rospy.loginfo("frame_id:" + str(self._odometry.header.frame_id))
        #
        # # Copying for simplicity
        # position = self._odometry.pose.pose.position
        # quat = self._odometry.pose.pose.orientation
        # rospy.loginfo("Point Position: [ %f, %f, %f ]" % (position.x, position.y, position.z))
        # rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]" % (quat.x, quat.y, quat.z, quat.w))
        #
        # # Also print Roll, Pitch, Yaw
        # euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        # rospy.loginfo("Euler Angles: %s" % str(euler))
        # rospy.loginfo("")

    def callback_laser(self, laserscan):
        """
        callback function used by a ros subscriber. Receives the laser and saves it to a list.
        :param pose: ros massage
        """
        self._laser_point_list = laserscan.ranges

        self._new_laser_data = True

        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", laserscan)
        # rospy.loginfo("%s", self._laser_point_list)
        # rospy.loginfo("%s", len(self._laser_point_list))

    def callback_map(self, occupancy_grid_msg):
        """

        :param occupancy_grid_msg:
        :return:
        """
        self._map = occupancy_grid_msg.data  # Total map

        # rospy.loginfo("MAP: " + str(self._map))
        self._occupancy_grid_msg = occupancy_grid_msg


if __name__ == '__main__':
    mcl = MonteCarlo()

    # Initialize Monte Carlo node
    rospy.init_node('monte_carlo', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, mcl.callback_map)
    while len(mcl._map) == 0:
        pass
    mcl.initialize_particles()

    # rospy.Subscriber("/RosAria/pose", Odometry, mcl.callback_odometry)
    # rospy.Subscriber("/scan", LaserScan, mcl.callback_laser)
    # mcl.publish_pose_array()
    rospy.spin()
