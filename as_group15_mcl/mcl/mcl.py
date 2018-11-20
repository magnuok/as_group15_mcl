import math
import numpy


class mcl:
    """
    Class description
    """

    _laser_point_list = []
    _odometry = []
    _map = []
    _particles = []

    def callback_odometry(self, message):
        """
        callback function used by a ros subscriber. Receives the odometry and saves it to a list.
        :param message: ros message
        """
        pass

    def callback_laser(self, message):
        """
        callback function used by a ros subscriber. Receives the laser and saves it to a list.
        :param message: ros massage
        """
        pass

    def _async_callback(self, particles):
        """
        used by calculate_new_particle_set to return particles from the asynchronous call. Saves the result to _particles
        :param particles: a new set of particles
        """
        self._particles = particles

    async def _calculate_new_particle_list(self, old_particles, odometry, laser_points, map):
        """
        calculates new particles based on Monte Carlo Localization.
        :param particles: The old list of particles
        :param odometry: The odometry since the last calculation
        :param laser_points: List of all the laser_points
        :param map: the map we want to localize the robot in.
        :return: the new list of particles.
        """
        # the new particles
        particle_list = []

        # the old_particles after applying the motion_model to them
        pose_list = []

        # the weights of the old_particles after applying the motion_model to them
        weight_list = []

        # for each particle in old_particle
        for particle in old_particles:
            # get new poses after applying the motion_model
            pose_list.append(type(self).sample_motion_model_odometry(odometry, particle))

            # get weights corresponding to the new pose_list
            weight_list.append(type(self).measurement_model(laser_points, pose_list, map))

        # return the new set of particles
        self._async_callback(particle_list)

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

    def _sample_motion_model(self, ):
        pass
