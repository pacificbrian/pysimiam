#
# (c) PySimiam Team 2013
#
# Contact person: Tim Fuchs <typograph@elec.ru>
#
# This class was implemented for the weekly programming excercises
# of the 'Control of Mobile Robots' course by Magnus Egerstedt.
#
from math import sqrt, sin, cos, atan2
import numpy
import asyncio, threading
from datetime import datetime
from core.ui import uiFloat
from core.helpers import Struct
from core.supervisor import Supervisor

from supervisors.quickbot import QuickBotSupervisor

from rhasspyhermes.nlu import NluIntent
from rhasspyhermes_app import EndSession, HermesApp

class QBFullSupervisor(QuickBotSupervisor):
    """QBFull supervisor implements the full switching behaviour for navigating labyrinths."""

    def add_controllers_no_blend(self, with_joystick=True):
        self.add_controller(self.gtg, \
                            (self.at_goal, self.hold), \
                            #(self.at_obstacle, self.wall), \
                            (self.approaching_obstacle, self.avoidobstacles))
        if with_joystick and self.joystick:
            self.add_controller(self.avoidobstacles, \
                                (self.at_goal, self.hold), \
                                (self.at_obstacle, self.wall), \
                                (self.safe, self.gtg), \
                                (self.joystick_override, self.joystick))
            self.add_controller(self.joystick, \
                                (self.all_clear, self.gtg))
        else:
            self.add_controller(self.avoidobstacles, \
                                (self.at_goal, self.hold), \
                                (self.at_obstacle, self.wall), \
                                (self.safe, self.gtg))
        self.add_controller(self.wall, \
                            (self.exit_follow_wall, self.gtg))

    def add_controllers_blended(self):
        self.add_controller(self.gtg, \
                            (self.at_goal, self.hold), \
                            (self.unsafe, self.blended))
        self.add_controller(self.blended, \
                            (self.at_goal, self.hold), \
                            (self.all_clear, self.gtg), \
                            (self.at_obstacle, self.avoidobstacles))
        self.add_controller(self.avoidobstacles, \
                            (self.at_goal, self.hold), \
                            (self.safe, self.blended), \
                            (self.all_clear, self.gtg))

    def rhasspy_init(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        app = HermesApp("Simiam", port=12183)
        loop.close()
        self.log("starting rhasspy app {}".format(app))

        @app.on_intent("GetTime")
        async def get_time(intent: NluIntent):
            now = datetime.now().strftime("%H %M")
            self.log("got intent! => GetTime")
            return EndSession(f"Hello from Simiam, It's now {now}")

        @app.on_intent("ActivateJoystick")
        async def activate_joystick(intent: NluIntent):
            self.current = self.gtg
            self.log("got intent! => JoystickActivated")
            return EndSession("Joystick Activated")

        return app

    def rhasspy_run(self):
        try:
            self.rhasspy.run() # blocking
        except ConnectionRefusedError:
            self.rhasspy = None
            pass

    def __init__(self, robot_pose, robot_info, options = None):
        """Create controllers and the state transitions"""
        QuickBotSupervisor.__init__(self, robot_pose, robot_info)

        self.extgoal = False

        if options is not None:
            try:
                self.parameters.goal.x = options.x
                self.parameters.goal.y = options.y
                self.extgoal = True
            except Exception:
                pass

        # Fill in some parameters
        self.parameters.sensor_poses = robot_info.ir_sensors.poses[:]
        self.parameters.ir_max = robot_info.ir_sensors.rmax
        self.parameters.direction = 'left'
        self.parameters.distance = 0.2
        self.parameters.joystick = Struct({"i_j":0, "i_x":0, "i_y":1, "inv_x":True, "inv_y":True})

        self.robot = robot_info
        self.rhasspy = self.rhasspy_init()
        threading.Thread(target=self.rhasspy_run, daemon=True).start()

        self.idle = True
        #Add controllers
        self.avoidobstacles = self.create_controller('AvoidObstacles', self.parameters)
        self.gtg = self.create_controller('GoToGoal', self.parameters)
        self.wall = self.create_controller('FollowWall', self.parameters)
        self.hold = self.create_controller('Hold', None)
        try:
            self.joystick = self.create_controller('joystick.JoystickController', \
                    (self.parameters.joystick, robot_info.wheels.max_velocity*robot_info.wheels.radius, \
                    robot_info.wheels.max_velocity*robot_info.wheels.radius/robot_info.wheels.base_length))
        except OSError:
            self.joystick = None
        self.num_transitions = 0

        # Define state transitions
        self.add_controller(self.hold,
                            (lambda: not self.at_goal(), self.gtg))
        self.add_controllers_no_blend()

        # Change and add additional transitions

        # Start in the 'go-to-goal' state
        self.current = self.gtg

        self.log("robot_size {} ir max {} joystick? {}".format(
            self.robot.wheels.base_length, self.robot.ir_sensors.rmax, self.joystick != None))

    def set_parameters(self,params):
        """Set parameters for itself and the controllers"""
        QuickBotSupervisor.set_parameters(self,params)
        self.gtg.set_parameters(self.parameters)
        self.avoidobstacles.set_parameters(self.parameters)
        self.wall.set_parameters(self.parameters)

    def stop(self):
        if self.rhasspy:
            self.rhasspy.stop()
            self.rhasspy = None

    def at_goal(self):
        """Check if the distance to goal is small"""
        at_goal = self.gtg.at_goal(self.parameters, self.robot_size)
        if at_goal and not self.idle:
            self.log("# transitions {}".format(self.num_transitions))
            self.idle = True
        return at_goal

    def joystick_override(self):
        """Check if the distance to obstacle is small"""
        guard = self.joystick and self.almost_crash()
        if guard:
            self.log("Threat detected: joystick override!")
        self.num_transitions += guard
        return guard

    def almost_crash(self):
        """Check if the distance to obstacle is small"""
        guard = self.distmin < (self.robot.ir_sensors.rmax / 8)
        self.num_transitions += guard
        return guard

    def at_obstacle(self):
        """Check if the distance to obstacle is small"""
        guard = self.distmin < (self.robot.ir_sensors.rmax / 2.15)
        if guard:
            ind_dmin = list(self.parameters.sensor_distances).index(self.distmin)
            if ind_dmin > 2:
                self.parameters.direction = 'right'
            elif ind_dmin < 2:
                self.parameters.direction = 'left'
            else:
                return False
            self.enter_fw_distance = self.distance_from_goal
            self.num_transitions += 1
        return guard

    def approaching_obstacle(self):
        """Check if the distance to obstacle is small"""
        guard = self.distmin < (self.robot.ir_sensors.rmax / 1.8)
        if guard:
            self.num_transitions += 1
            self.idle = False
        return guard

    def unsafe(self):
        # see week7_solved
        guard = self.distmin < (self.robot.ir_sensors.rmax / 1.4)
        if guard:
            self.num_transitions += 1
            self.idle = False
        return guard

    def safe(self):
        # see week7_solved
        # rmin?
        guard = self.distmin > (self.robot.ir_sensors.rmax / 1.25)
        self.num_transitions += guard
        return guard

    def all_clear(self):
        """Check if the distance to obstacle is large"""
        # rmin?
        guard = self.distmin > (self.robot.ir_sensors.rmax / 1.1)
        self.num_transitions += guard
        return guard

    def progress_made(self):
        if (self.distance_from_goal + 0.08) < self.enter_fw_distance:
            #self.log("FW Progress Made! {} < {}".format(self.distance_from_goal, self.enter_fw_distance))
            return True
        return False

    def detach_wall(self):
        """Check if detatching from the wall makes sense (the goal is on the right side"""
        goal_angle = self.gtg.get_heading_angle(self.parameters)
        wall_angle = self.wall.get_heading_angle(self.parameters)
        #uao = self.avoidobstacles.get_heading(self.parameters)
        #ugtg = self.gtg.get_heading(self.parameters)
        #theta = numpy.dot(numpy.transpose(uao), ugtg)

        guard = False
        if goal_angle > wall_angle and self.parameters.direction == "right":
            guard = True
        if goal_angle < wall_angle and self.parameters.direction == "left":
            guard = True
        #self.log("FW Detach? {} {}".format(goal_angle, wall_angle))
        return guard

    def exit_follow_wall(self):
        guard = self.progress_made() and self.detach_wall()
        self.num_transitions += guard
        return guard

    def process_state_info(self, state):
        """Update state parameters for the controllers and self"""

        QuickBotSupervisor.process_state_info(self,state)

        # The pose for controllers
        self.parameters.pose = self.pose_est

        # Distance to the goal
        self.distance_from_goal = self.gtg.get_goal_distance(self.parameters)

        # Sensor readings in real units
        self.parameters.sensor_distances = self.get_ir_distances()

        # Distance to the closest obstacle
        self.distmin = min(self.parameters.sensor_distances)

    def draw_foreground(self, renderer):
        """Draw controller info"""
        QuickBotSupervisor.draw_foreground(self,renderer)

        # Make sure to have all headings:
        renderer.set_pose(self.pose_est)
        arrow_length = self.robot_size*5

        # Ensure the headings are calculated

        # Draw arrow to goal
        if self.current == self.gtg:
            goal_angle = self.gtg.get_heading_angle(self.parameters)
            renderer.set_pen(0x00FF00)
            renderer.draw_arrow(0,0,
                arrow_length*cos(goal_angle),
                arrow_length*sin(goal_angle))

        # Draw arrow away from obstacles
        elif self.current == self.avoidobstacles:
            away_angle = self.avoidobstacles.get_heading_angle(self.parameters)
            renderer.set_pen(0xCC3311)
            renderer.draw_arrow(0,0,
                arrow_length*cos(away_angle),
                arrow_length*sin(away_angle))

        elif self.current == self.wall:

            # Draw vector to wall:
            renderer.set_pen(0x0000FF)
            renderer.draw_arrow(0,0,
                self.wall.to_wall_vector[0],
                self.wall.to_wall_vector[1])
            # Draw
            renderer.set_pen(0xFF00FF)
            renderer.push_state()
            renderer.translate(self.wall.to_wall_vector[0], self.wall.to_wall_vector[1])
            renderer.draw_arrow(0,0,
                self.wall.along_wall_vector[0],
                self.wall.along_wall_vector[1])
            renderer.pop_state()

            # Draw heading (who knows, it might not be along_wall)
            renderer.set_pen(0xFF00FF)
            renderer.draw_arrow(0,0,
                arrow_length*cos(self.wall.heading_angle),
                arrow_length*sin(self.wall.heading_angle))

    def get_ui_description(self,p = None):
        """Returns the UI description for the docker"""
        if p is None:
            p = self.parameters

        ui =   [('goal', [('x',uiFloat(p.goal.x,0.1)), ('y',uiFloat(p.goal.y,0.1))]),
                ('velocity', [('v',uiFloat(p.velocity.v,0.1))]),
                (('gains',"PID gains"), [
                    (('kp','Proportional gain'), uiFloat(p.gains.kp,0.1)),
                    (('ki','Integral gain'), uiFloat(p.gains.ki,0.1)),
                    (('kd','Differential gain'), uiFloat(p.gains.kd,0.1))])]

        if self.extgoal:
            return ui[1:]
        else:
            return ui
