import numpy
import math

"""
Contains an implementation of the sample_motion_model_odometry as described in
https://docs.google.com/document/d/1EY1oFbIIb8Ac_3VP40dt_789sVhlVjovoN9eT78llkQ/edit
"""


def sample_motion_model_odometry(u, x_last):
    """
    :param u: a tuple of movement (x, y, theta) where x and y is distance in the plane, and theta is the
    rotation.
    :param x_last: a tuple of old position (x, y, theta) where x and y is position in the plane, and theta is the
    orientation
    :return: returns a tuple which represents the new position (x, y, theta) where x and y is position in the plane,
    and theta is the orientation
    """

    # TODO: find out what these values should be
    # constants
    ALFA_1 = 0.1;
    ALFA_2 = 0.1;
    ALFA_3 = 0.1;
    ALFA_4 = 0.1;

    delta_rot_1 = numpy.arctan2(u[1], u[0])
    delta_trans = math.sqrt(u[0] ^ 2 + u[1] ^ 2)
    delta_rot_2 = u[2] - delta_rot_1

    delta_rot_1_hat = delta_rot_1 - sample(abs(ALFA_1* delta_rot_1 + ALFA_2 * delta_trans))
    delta_trans_hat = delta_trans - sample(abs(ALFA_3 * delta_trans + ALFA_4 * (delta_rot_1 + delta_rot_2)))
    delta_rot_2_hat = delta_rot_2 - sample(abs(ALFA_1 * delta_rot_2 + ALFA_2 * delta_trans))

    x = x_last[0] + delta_trans_hat * math.cos(x_last[2] + delta_rot_1)
    y = x_last[1] + delta_trans_hat * math.sin(x_last[2] + delta_rot_1)
    theta = x_last[2] + delta_rot_1_hat + delta_rot_2_hat

    return x, y, theta


def sample(standard_deviation):
    """
    :param standard_deviation: standard deviation.
    :return: A number chosen randomly from the normal distribution with center in 0 and standard deviation =
    standard_deviation
    """
    return numpy.random.normal(loc=0, scale=standard_deviation, size=None)
