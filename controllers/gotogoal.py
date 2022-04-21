#
# (c) PySimiam Team 2013
#
# Contact person: Tim Fuchs <typograph@elec.ru>
#
# This class was implemented as a weekly programming excercise
# of the 'Control of Mobile Robots' course by Magnus Egerstedt.
#
import math
import numpy

from controllers.pid_controller import PIDController

class GoToGoal(PIDController):
    """Go-to-goal steers the robot to a predefined position in the world."""
    def __init__(self, params):
        """Initialize internal variables"""
        PIDController.__init__(self,params)

    def normalize_angle(self, theta):
        return ((theta + math.pi)%(2*math.pi)) - math.pi

    def get_goal_distance(self, state):
        y = state.goal.y - state.pose.y
        x = state.goal.x - state.pose.x
        return math.sqrt((y**2)+(x**2))

    def at_goal(self, state, base_size):
        return self.get_goal_distance(state) < (base_size / 3)

    # Let's overwrite this way:
    def get_heading_angle(self, state):
        """Get the direction from the robot to the goal as a vector."""

        # The goal:
        x_g, y_g = state.goal.x, state.goal.y

        # The robot:
        x_r, y_r, theta = state.pose

        # Where is the goal in the robot's frame of reference?
        desired = math.atan2(y_g - y_r, x_g - x_r)
        return self.normalize_angle(desired - theta)

    def get_heading(self,state):
        """Get the direction in which the controller wants to move the robot
        as a vector in the robot's frame of reference.

        :return: a numpy array [x, y, z] with z = 1.
        """

        goal_angle = self.get_heading_angle(state)
        return numpy.array([math.cos(goal_angle),math.sin(goal_angle),1])

    def get_gain(self, state, error, a):
        # error used is distance to goal
        # work in progress
        error = self.get_goal_distance(state) * 10
        error = abs(error)
        gain = (1 - math.exp(-a * error * error)) / error
        #self.log("gtg get_gain {} err {} a {}".format(gain, error, a))
        return gain

    def execute(self, state, dt):
        v, w = PIDController.execute(self, state, dt)
        return v, w
