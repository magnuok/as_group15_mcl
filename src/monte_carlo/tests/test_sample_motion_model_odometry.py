import unittest
from src.monte_carlo import MonteCarlo
import math

class Test_sample_motion_model_odometry(unittest.TestCase):


    def setUp(self):
        self.mcl = MonteCarlo(testing=True)
        self.odometry_list = [(0,0,math.pi), (1, 1, math.pi/2)]
        self.particle_list = [(1,1, 2), (1, 5, math.pi), (22, 51515, 0)]


    def tearDown(self):
        pass


    def test_receives_new_odometry(self):
        for odometry in self.odometry_list:
            for particle in self.particle_list:
                self.assertNotEqual(MonteCarlo.sample_motion_model_odometry(odometry, particle), None)


    def test_receives_correct_format(self):
        for odometry in self.odometry_list:
            for particle in self.particle_list:
                self.assertTrue(len(MonteCarlo.sample_motion_model_odometry(odometry, particle)), 3)

    # TODO: add test that check that the retuned angle is between 0 and 2pi (?)

if __name__ == '__main__':
    unittest.main()