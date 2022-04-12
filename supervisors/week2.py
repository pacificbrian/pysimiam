#
# (c) PySimiam Team 2013
#
# Contact person: Tim Fuchs <typograph@elec.ru>
#
# This class was implemented for the weekly programming excercises
# of the 'Control of Mobile Robots' course by Magnus Egerstedt.
#
from numpy import array, dot, negative
from scipy import interpolate
from math import pi, sin, cos, log1p, sqrt, atan2

from core.supervisor import Supervisor
from core.helpers import Struct
from core.pose import Pose
from core.simobject import Path

class QuickBotSupervisor(Supervisor):
    """The QuickBotSupervisor inherits from the superclass 'supervisor.Supervisor' to implement detailed calculations for any inheriting QuickBot supervisor.

    Most importantly, the QuickBotSupervisor object implements the system functions necessary to operate a QuickBot, namely the uni2diff unicycle to differential motion model conversion, the Jacobian problem, and any other computationally complex interface.

    The UI may use the get_parameters function interface to create docker windows for real-time update of the PID parameters. This is an advanced implementation and is not required for students to properly implement their own supervisors."""

    ir_interp = None

    def __init__(self, robot_pose, robot_info):
        """Initialize internal variables"""
        Supervisor.__init__(self, robot_pose, robot_info)

        # initialize memory registers
        self.prev_left_ticks  = robot_info.wheels.left_ticks
        self.prev_right_ticks = robot_info.wheels.right_ticks

        # Create tracker
        self.tracker = Path(robot_pose, 0)

        # Create & set the controller
        self.current = self.create_controller('GoToAngle', self.parameters)

        # for ir readings conversion
        dis_m = [0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.25, 0.30]
        dis_adc = [917, 783, 683, 583, 517, 467, 425, 358, 308, 268, 242, 217, 167, 133]
        dis_adc_new = self.ir_readings_transform(dis_adc)
        ir_interp = interpolate.interp1d(array(dis_adc_new), array(dis_m), kind='linear', assume_sorted=True)

        self.ir_interp = ir_interp

    def ir_readings_transform(self, readings):
        adjusted = negative(readings)
        return adjusted.tolist()

    def init_default_parameters(self):
        """Sets the default PID parameters, goal, and velocity"""
        p = Struct()
        p.goal = 45.0
        p.velocity = 0.2
        p.pgain = 3.0

        self.parameters = p

    def get_ui_description(self,p = None):
        """Returns the UI description for the docker"""
        if p is None:
            p = self.parameters

        return [(('goal', 'Target angle'), p.goal),
                ('velocity', p.velocity),
                (('pgain',"Proportional gain"), p.pgain)]

    def set_parameters(self,params):
        """Set parameters for itself and the controllers"""
        self.parameters.goal = params.goal
        self.parameters.velocity = params.velocity
        self.parameters.pgain = params.pgain
        self.current.set_parameters(self.parameters)

    def uni2diff(self,uni):
        """Convert from unicycle model to differential model"""
        (v,w) = uni

        #Insert Week 2 Assignment Code Here

        R = self.robot.wheels.radius
        L = self.robot.wheels.base_length

        vl = (2*v - w*L) / (R*2)
        vr = (2*v + w*L) / (R*2)

        #End Week 2 Assignment Code

        return (vl,vr)

    def estimate_pose(self):
        """Update self.pose_est using odometry"""

        #Insert Week 2 Assignment Code Here
        R = self.robot.wheels.radius
        L = self.robot.wheels.base_length
        N = self.robot.wheels.ticks_per_rev

        # Get tick updates
        delta_lticks = self.robot.wheels.left_ticks - self.prev_left_ticks
        delta_rticks = self.robot.wheels.right_ticks - self.prev_right_ticks

        D_left = 2*pi*R*(delta_lticks / N)
        D_right = 2*pi*R*(delta_rticks / N)
        D_average = (D_left + D_right) / 2

        # Save the wheel encoder ticks for the next estimate
        self.prev_left_ticks += delta_lticks
        self.prev_right_ticks += delta_rticks

        #Get the present pose estimate
        x, y, theta = self.pose_est

        theta_new = theta + (D_right - D_left) / L
        x_new = x + D_average * cos(theta)
        y_new = y + D_average * sin(theta)

        #End Week 2 Assignment Code

        return Pose(x_new, y_new, (theta_new + pi)%(2*pi)-pi)

    def get_ir_distances(self, debug=False):
        """Converts the IR distance readings into a distance in meters"""

        #Insert Week 2 Assignment Code Here

        readings = self.ir_readings_transform(self.robot.ir_sensors.readings)
        ir_distances = self.ir_interp(readings)
        if debug:
            self.log("ir readings {} {}".format(readings, ir_distances))
        #End Assignment week2
        return ir_distances

    def execute(self, robot_info, dt):
        """Inherit default supervisor procedures and return unicycle model output (x, y, theta)"""
        output = Supervisor.execute(self, robot_info, dt)
        self.tracker.add_point(self.pose_est)
        return self.uni2diff(output)

    def process_state_info(self, state):
        """Update state parameters for the controllers and self"""

        Supervisor.process_state_info(self,state)

        self.parameters.pose = self.pose_est

    def draw_background(self, renderer):
        """Draw a circular goal and path"""

        # Draw goal
        renderer.set_pose(Pose(self.pose_est.x,self.pose_est.y,pi*self.parameters.goal/180))
        renderer.set_pen(0)
#        renderer.draw_line(0.03,0,100,0)
        renderer.draw_arrow(0.05,0,0.5,0,close=True)

        # Draw robot path
        self.tracker.draw(renderer)

    def draw_foreground(self, renderer):
        """Draw ir_sensors"""

        renderer.set_pose(self.pose_est)
        renderer.set_brush(0)
        renderer.draw_ellipse(0,0,0.01,0.01)

        # Draw IR distances
        crosses = array([ dot(p.get_transformation(), [d,0,1])
                          for d, p in zip(self.get_ir_distances(), self.robot.ir_sensors.poses) ] )

        renderer.set_pen(0)
        for c in crosses:
            x,y,z = c

            renderer.push_state()
            renderer.translate(x,y)
            renderer.rotate(atan2(y,x))

            renderer.draw_line(0.01,0.01,-0.01,-0.01)
            renderer.draw_line(0.01,-0.01,-0.01,0.01)

            renderer.pop_state()

    def get_controller_state(self):
        return self.parameters
