import unittest
from src.monte_carlo import MonteCarlo
import math

class Test_low_variance_sampler(unittest.TestCase):


    def setUp(self):
        self.mcl = MonteCarlo(testing=True)
        self.particle_list = [(1,1, 2), (1, 5, math.pi), (22, 51515, 0)]
        self.weights = [1,1,1]


    def tearDown(self):
        pass

    def test_receive_same_amount_of_particles(self):
        self.assertTrue(MonteCarlo.low_variance_sampler(particles=self.particle_list, weights=self.weights))


    # TODO: what other test do we need?


if __name__ == '__main__':
    unittest.main()