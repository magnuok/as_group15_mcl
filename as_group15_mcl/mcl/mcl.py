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
