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
from controllers.avoidobstacles import AvoidObstacles
from controllers.gotogoal import GoToGoal

class Blending(GoToGoal, AvoidObstacles):
    """A controller blending go-to-goal and avoid-obstacles behaviour"""
    def __init__(self, params):
        """Initialize internal variables"""
        PIDController.__init__(self,params)

        # These two angles are used by the supervisor
        # to debug the controller's behaviour, and contain
        # the headings as returned by the two subcontrollers.
        self.goal_angle = 0
        self.away_angle = 0

    def set_parameters(self, params):
        """Set PID values and sensor poses.

        The params structure is expected to have sensor poses in the robot's
        reference frame as ``params.sensor_poses``.
        """
        AvoidObstacles.set_parameters(self,params)

    def get_heading_angle(self, state):
        return PIDController.get_heading_angle(self,state)

    def get_total_sensor_distance(self,state):
        dist = 0
        for d in state.sensor_distances:
            dist += d
        return dist

    def blend_factor(self, state, beta):
        # TODO work in progress for blend factor
        D = self.get_total_sensor_distance(state)
        a1 = round(1 - math.exp(-beta * D), 3)
        a2 = math.exp(-0.2 / min(state.sensor_distances) + 1)
        return (a1, a2)

    def get_heading(self, state):
        """Blend the two heading vectors"""

        # Get the outputs of the two subcontrollers
        u_ao = AvoidObstacles.get_heading(self,state)
        self.away_angle = math.atan2(u_ao[1],u_ao[0])
        #u_ao = numpy.array([math.cos(self.away_angle),math.sin(self.away_angle),1])
        # XXX compare above with first u_ao

        self.goal_angle = GoToGoal.get_heading_angle(self,state)
        u_gtg = numpy.array([math.cos(self.goal_angle),math.sin(self.goal_angle),1])
        #u_gtg = GoToGoal.get_heading(self,state)

        # Week 5 Assigment Code goes here:
        beta = 0.01
        # alpha should be small as closer to obstacles
        #a = 0.3
        a1, a2 = self.blend_factor(state, beta)
        a = a2
        #self.log("blend function {} {} for D {}".format(a, beta, D))
        u = a*u_gtg + (1-a)*u_ao

        # End Week 5 Assigment

        return u

    def execute(self, state, dt):

        v, w = PIDController.execute(self, state, dt)

        # Week 5 Assigment Code goes here:
        # End Week 5 Assigment

        return v, w
