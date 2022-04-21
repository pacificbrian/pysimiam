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

from core.pose import Pose
from controllers.pid_controller import PIDController

class FollowWall(PIDController):
    """Follow walls is a controller that keeps a certain distance
    to the wall and drives alongside it in clockwise or counter-clockwise
    fashion."""
    def __init__(self, params):
        '''Initialize internal variables'''
        PIDController.__init__(self,params)

        # This variable should contain a list of vectors
        # calculated from the relevant sensor readings.
        # It is used by the supervisor to draw & debug
        # the controller's behaviour
        self.vectors = []
        self.weights = [0.185, 0.20, 0.23, 0.20, 0.185]

        # Normalizing weights
        ws = sum(self.weights)
        self.weights = [w/ws for w in self.weights]
        self.log("{} sensors with weights {} {}".format(
                 len(self.weights), self.weights, round(sum(self.weights),2)))

    def restart(self):
        """Reset internal state"""
        PIDController.restart(self)

    def set_parameters(self, params):
        """Set PID values, sensor poses, direction and distance.
        
        The params structure is expected to have sensor poses in the robot's
        reference frame as ``params.sensor_poses``, the direction of wall
        following (either 'right' for clockwise or 'left' for anticlockwise)
        as ``params.direction`` and the desired distance to the wall 
        to maintain as ``params.distance``.
        """
        PIDController.set_parameters(self,params)

        self.sensor_poses = params.sensor_poses
        self.direction = params.direction
        self.distance = params.distance

    def get_heading(self, state):
        """Get the direction along the wall as a vector."""
        
        # Week 6 Assignment:
        
        # Calculate vectors for the sensors
        self.vectors = []
        # Calculate vectors for the sensors
        if state.direction == 'left': # 0-2
            d, i = min( list(zip(state.sensor_distances[:3],[0,1,2])) )
            if i == 0 or (i == 1 and state.sensor_distances[0] <= state.sensor_distances[2]):
                i, j, k = 1, 0, 2
            else:
                i, j, k = 2, 1, 0
        else : # 2-4
            d, i = min( list(zip(state.sensor_distances[2:],[2,3,4])) )
            if i == 4 or (i == 3 and state.sensor_distances[4] <= state.sensor_distances[2]):
                i, j, k = 3, 4, 2
            else:
                i, j, k = 2, 3, 4

        p_front = Pose(state.sensor_distances[i]) >> self.sensor_poses[i]
        p_back = Pose(state.sensor_distances[j]) >> self.sensor_poses[j]

        self.vectors = numpy.array([(p_front.x,p_front.y,1), (p_back.x, p_back.y, 1)])
        p2 = self.vectors[0]
        p1 = self.vectors[1]

        #self.log("follow wall v {} {} {} idx {}".format(p2, p1, p2 - p1, (i,j)))
        self.along_wall_vector = (p2 - p1)
        aw_v_prime = self.along_wall_vector / (2*numpy.linalg.norm(p2 - p1))

        p2 = p_front
        p1 = p_back
        ds = ((p2.x-p1.x)**2 + (p2.y-p1.y)**2)
        ms = (p2.x*p1.y - p2.y*p1.x)
        self.to_wall_vector = numpy.array([(p1.y-p2.y)*ms/ds,(p2.x-p1.x)*ms/ds,1])
        # TODO this is simpler and looks same result:
        #self.to_wall_vector = ((p2 + p1) / 3)
        #self.log("toward wall {} {}".format(p2, p1))

        # Calculate and return the heading vector:                            
        weight = 0.85
        distance_vector =  self.to_wall_vector - self.distance * (self.to_wall_vector / numpy.linalg.norm(self.to_wall_vector))
        self.to_wall_vector = distance_vector
        return aw_v_prime + (1 - weight) * distance_vector
        # End Week 6 Assignment
