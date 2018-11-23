import unittest
from src.monte_carlo import MonteCarlo
import math

class Test_update_particle_list(unittest.TestCase):


    def setUp(self):
        self.mcl = MonteCarlo(testing=True)
        self.odometry_list = [(0,0,math.pi), (1, 1, math.pi/2)]
        self.particle_list = [(1,1, 2), (1, 5, math.pi), (22, 51515, 0)]
        self.map = []
        self.laser_points = []


    def tearDown(self):
        pass

    # TODO: get mock of map and laser_points
    # TODO: check that it returns a list
    # TODO: check that it returns something that have particles in it
    # TODO: check that it doesn't use too long time?
    # TODO: check...

if __name__ == '__main__':
    unittest.main()