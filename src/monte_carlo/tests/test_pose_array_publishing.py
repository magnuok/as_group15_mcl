import unittest
from src.monte_carlo import MonteCarlo
import math

class Test_pose_array_publishing(unittest.TestCase):


    def setUp(self):
        self.mcl = MonteCarlo(testing=True)


        self.particle_list = [(1,1, 2), (1, 5, math.pi), (22, 51515, 0)]
        self.mcl._update_pose_array(self.particle_list)



    def tearDown(self):
        pass

    def test_receive_same_amount_of_particles(self):
        self.assertTrue(MonteCarlo.low_variance_sampler(predicted_particles=self.particle_list, weights=self.weights))


    # TODO: what other test do we need?


if __name__ == '__main__':
    unittest.main()