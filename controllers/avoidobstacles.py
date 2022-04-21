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

from core.pose import Pose

from controllers.pid_controller import PIDController

class AvoidObstacles(PIDController):
    """Avoid obstacles is an example controller that checks the sensors
       for any readings, constructs 'obstacle' vectors and directs the robot
       in the direction of their weighted sum."""
    def __init__(self, params):
        '''Initialize internal variables'''
        PIDController.__init__(self,params)

    def set_parameters(self, params):
        """Set PID values and sensor poses.

        The params structure is expected to have sensor poses in the robot's
        reference frame as ``params.sensor_poses``.
        """
        PIDController.set_parameters(self,params)

        self.sensor_poses = params.sensor_poses

        # Now we know the poses, it makes sense to also
        # calculate the weights
        #self.weights = [(math.cos(p.theta)+1.5) for p in self.sensor_poses]
        #self.weights = [1.0, 1.0, 0.5, 1.0, 1.0]
        self.weights = [0.185, 0.20, 0.23, 0.20, 0.185]

        # Normalizing weights
        ws = sum(self.weights)
        self.weights = [w/ws for w in self.weights]
        self.log("{} sensors with weights {} {}".format(
                 len(self.weights), self.weights, round(sum(self.weights),2)))

    def get_heading(self, state):
        """Get the direction away from the obstacles as a vector.

        :return: a numpy array [x, y, z] with z = 1.
        """

        # Calculate heading:
        x, y = 0, 0
        for d,p,w in zip(state.sensor_distances, self.sensor_poses, self.weights):
            pose = Pose(d) >> p
            x += pose.x*w
            y += pose.y*w

        # TODO above is good but best is to consider current direction of
        # robot, having dynamic weight adjustment (ignore some sensors)

        return numpy.array([x, y, 1])

    def get_gain(self, state, error, e):
        # error here is distance to obstacle
        # work in progress
        error = 0
        error = abs(error)
        gain = (1/error) * (1 / (error * error + e))
        #self.log("ao get_gain {} err {} e {}".format(gain, error, e))
        return gain

    def execute(self, state, dt):
        v, w = PIDController.execute(self, state, dt)

        #dmin = min(state.sensor_distances)
        #v *= ((dmin - 0.04)/0.26)**2

        return v, w
