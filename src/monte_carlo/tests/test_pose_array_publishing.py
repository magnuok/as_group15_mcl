import unittest
from src.monte_carlo import MonteCarlo
import math
import time

class Test_pose_array_publishing(unittest.TestCase):


    def setUp(self):
        self.mcl = MonteCarlo(testing=True)
        self.mcl._initialize_publisher()

        for i in range (0,100):
            particle = (1 ,1 ,math.pi)
            self.mcl._particles.append(particle)



    def test_response_time(self):
        self.mcl._update_pose_array(self.mcl._particles)
        start_time = time.time()
        self.mcl._publish_pose_array(self.mcl._pose_array)
        elapsed_time = time.time() - start_time
        while elapsed_time < 2:
            elapsed_time = time.time() - start_time
        self.assertTrue(elapsed_time<10)

    def tearDown(self):
        pass

    def test_receive_same_amount_of_particles(self):
        pass
        # self.assertTrue()


    # TODO: what other test do we need?


if __name__ == '__main__':
    unittest.main()