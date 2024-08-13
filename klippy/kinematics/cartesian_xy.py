# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
from configfile import ConfigWrapper
from extras.homing import Homing
from stepper import PrinterRail
from toolhead import Move, ToolHead
from . import idex_modes

# Conversion to XY only:
# The backend works in (X, Y, Z, E) coordinates (X, Y, Z, Extruder)
# In gcode.py: `Coord = collections.namedtuple('Coord', ('x', 'y', 'z', 'e'))`
# Z cannot be eliminated but only turned into no-op

# Was: USED_RAILS_LABELS = 'xyz'
# iterates over [stepper_<x>] sections in config
USED_RAILS_LABELS = "xy"

# X, Y, Z, E (X, Y, Z, Extruder)
# AXIS_IDX = (0, 1, 2)
AXIS_IDX = (0, 1)

class CartKinematicsXY:

    def __init__(self, toolhead: ToolHead, config: ConfigWrapper):
        self.printer = config.get_printer()
        # Setup axis rails
        self.dual_carriage_axis = None
        self.dual_carriage_rails = []

        self.rails: list[PrinterRail] = []
        for rail_label in USED_RAILS_LABELS:
            printer_rail = stepper.LookupMultiRail(config.getsection('stepper_' + rail_label))
            self.rails.append(printer_rail)

        # self.rails: list[PrinterRail] = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
        #               for n in USED_RAILS_LABELS]

        for rail, axis in zip(self.rails, USED_RAILS_LABELS):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())

        ranges = [r.get_range() for r in self.rails]
        # Was:
        # self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        # self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)

        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], z=0.0, e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], z=0.0, e=0.)

        self.dc_module = None

        #TODO: eliminate Z from this section for completeness
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            dc_axis = dc_config.getchoice('axis', ['x', 'y'])
            self.dual_carriage_axis = {'x': 0, 'y': 1}[dc_axis]
            # setup second dual carriage rail
            self.rails.append(stepper.LookupMultiRail(dc_config))
            self.rails[3].setup_itersolve('cartesian_stepper_alloc',
                                          dc_axis.encode())
            dc_rail_0 = idex_modes.DualCarriagesRail(
                    self.rails[self.dual_carriage_axis],
                    axis=self.dual_carriage_axis, active=True)
            dc_rail_1 = idex_modes.DualCarriagesRail(
                    self.rails[3], axis=self.dual_carriage_axis, active=False)
            self.dc_module = idex_modes.DualCarriages(
                    dc_config, dc_rail_0, dc_rail_1,
                    axis=self.dual_carriage_axis)

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()

        # TODO: check if needed
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        rails = self.rails

        if self.dc_module:
            primary_rail = self.dc_module.get_primary_rail().get_rail()
            rails = (rails[:self.dc_module.axis] +
                     [primary_rail] + rails[self.dc_module.axis+1:])

        positions = [stepper_positions[rail.get_name()] for rail in rails]
        return positions

    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
        for axis in homing_axes:
            if self.dc_module and axis == self.dc_module.axis:
                rail = self.dc_module.get_primary_rail().get_rail()
            else:
                rail = self.rails[axis]
            self.limits[axis] = rail.get_range()

    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)

    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)

    def home(self, homing_state: Homing):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            if self.dc_module is not None and axis == self.dual_carriage_axis:
                self.dc_module.home(homing_state)
            else:
                self.home_axis(homing_state, axis, self.rails[axis])

    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3

    def _check_endstops(self, move):
        end_pos = move.end_pos

        # for i in (0, 1, 2):
        for i in AXIS_IDX:
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move: Move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)

        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return

        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip(USED_RAILS_LABELS, self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CartKinematicsXY(toolhead, config)
