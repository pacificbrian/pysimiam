#
# (c) PySimiam Team 2013
#
# Contact person: Tim Fuchs <typograph@elec.ru>
#
# This class was implemented for the weekly programming excercises
# of the 'Control of Mobile Robots' course by Magnus Egerstedt.
#
import math
import numpy

from controllers.pid_controller import PIDController

class AvoidObstacles(PIDController):
    """Avoid obstacles is an example controller that checks the sensors
       for any readings, constructs 'obstacle' vectors and directs the robot
       in the direction of their weightd sum."""
    def __init__(self, params):
        '''Initialize internal variables'''
        PIDController.__init__(self,params)

        # This variable should contain a list of vectors
        # calculated from sensor readings. It is used by
        # the supervisor to draw & debug the controller's
        # behaviour
        self.vectors = []

    def set_parameters(self, params):
        """Set PID values and sensor poses.

        The params structure is expected to have sensor poses in the robot's
        reference frame as ``params.sensor_poses``.
        """
        PIDController.set_parameters(self,params)

        self.sensor_poses = params.sensor_poses

        # Week 4 assigment

        # Set the weights here -
        # played with static and based on p.theta
        self.weights = [1]*len(self.sensor_poses)
        self.weights[0] = 0.09
        self.weights[1] = 0.25
        self.weights[2] = 0.28
        self.weights[3] = 0.25
        self.weights[4] = 0.09
        self.weights = [(math.cos(p.theta/2)+0.20) for p in self.sensor_poses]
        # normalize weights
        self.weights = [w/sum(self.weights) for w in self.weights]

        self.log("{} sensors with weights {} {}".format(
                 len(self.weights), self.weights, round(sum(self.weights),2)))

    def get_heading(self, state):
        """Get the direction away from the obstacles as a vector."""

        # Week 4 Assignment:
        # Calculate vectors:
        self.vectors = []

        # Calculate weighted sum:
        heading = [0, 0, 0]
        # TODO: calculate weights dynamically based on heading (sensor threat)
        for dist, pose, weight in zip(state.sensor_distances, self.sensor_poses, self.weights):
            d = numpy.array([dist, 0, 1])
            R = pose.get_transformation()
            v = numpy.dot(R, d)
            weighted_v = v * weight
            #self.log("sensor point {} {} {} {}".format(dist, R, v, weighted_v))
            self.vectors.append(v)
            heading += weighted_v

        # End Week 4 Assignment

        return heading
