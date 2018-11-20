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


class MonteCarlo:
    """
    Class description
    """

    _laser_point_list = []  # List with laserscan. Scanning [pi, 0]. 512 scans
    _odometry = ()  # Contains the new odometry tuppel = (x,y,theta)
    _map = []   # Contains list of cells in map
    _particles = []   # List with particle tuples = (x,y, theta)
    _pose_array = PoseArray()

    _occupancy_grid_msg = OccupancyGrid()


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
        #rospy.loginfo(self._odometry)

        #self._odometry = pose

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
        #rospy.loginfo("Euler Angles: %s" % str(euler))
        # rospy.loginfo("")


    def callback_laser(self, laserscan):
        """
        callback function used by a ros subscriber. Receives the laser and saves it to a list.
        :param pose: ros massage
        """
        self._laser_point_list = laserscan.ranges
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", laserscan)
        #rospy.loginfo("%s", self._laser_point_list)
        #rospy.loginfo("%s", len(self._laser_point_list))


    def _async_callback(self, particles):
        """
        used by calculate_new_particle_set to return particles from the asynchronous call. Saves the result to _particles
        :param particles: a new set of particles
        """
        self._particles = particles


    def _calculate_new_particle_list(self, particles, odometry, laser_points, map):
        """
        calculates new particles based on Monte Carlo Localization.
        :param particles: The old list of particles
        :param odometry: The odometry since the last calculation
        :param laser_points: List of all the laser_points
        :param map: the map we want to localize the robot in.
        :return: the new list of particles.
        """
        pass

    # def mcl(X_previous, u, z, map):
    #
    #     X = []
    #
    #     X_bar = []
    #     W = []
    #     # X_bar = {}
    #     M = 100
    #
    #     for m in range(0, len(X_previous)):
    #         x = sample_motion_model(u, X_previous[m])
    #         w = measurement_model(z, x, map)
    #         X_bar.append(x)
    #         W.append(w)
    #         # X_bar[x] = w
    #
    #     # Before resampling:
    #     # Check the number of effective particles
    #     n_eff = 1. / sum([w * w for w in W])
    #     if n_eff < M / 2 and set(X) != set(X_previous):
    #         X = low_variance_sample(X_bar, W)
    #     else:
    #         X = X_previous
    #     return X

    def _sample_motion_model(self, ):
        pass

    def callback_map(self, occupancy_grid_msg):
        """

        :param occupancy_grid_msg:
        :return:
        """
        self._map = occupancy_grid_msg.data  # Total map

        rospy.loginfo("MAP: " +str(self._map))
        self._occupancy_grid_msg = occupancy_grid_msg


    def initialize_particles(self):
        """

        :param:
        :return:
        """

        # Dimentions
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

        #rospy.loginfo("_map" + str(self._map))
        free_space_list = []  # Only free space
        initial_particle_placement = []  # list with initial cells

        # add the element numbers that are free
        for i in range(0, len(self._map)):
            if self._map[i] != 100 and self._map[i] != -1:
                free_space_list.append(i)
        rospy.loginfo("Free space list: " +str(free_space_list))

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

    def publish_pose_array(self):
        pub = rospy.Publisher('/PoseArray', PoseArray, queue_size=10)
        rate = rospy.Rate(2)  # Every 1/f seconds

        self._pose_array.header.frame_id = "map"  # Set frame_id to be recognized in rviz

        # While not shutdown: publish a PoseArray containing particles to rviz
        while not rospy.is_shutdown():
            # Create new pose from article to publish to rviz
            for particle in self._particles:
                pose = self._create_pose(particle)
                self._pose_array.poses.append(pose)  # Add to pose_array

            pub.publish(self._pose_array)
            rate.sleep()

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


if __name__ == '__main__':
    mcl = MonteCarlo()

    # Initialize Monte Carlo node
    rospy.init_node('monte_carlo', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, mcl.callback_map)
    while len(mcl._map) == 0:
        pass
    mcl.initialize_particles()

    #rospy.Subscriber("/RosAria/pose", Odometry, mcl.callback_odometry)
    #rospy.Subscriber("/scan", LaserScan, mcl.callback_laser)
    #mcl.publish_pose_array()
    rospy.spin()



