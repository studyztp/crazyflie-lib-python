#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2023 Bitcraze AB
#
#  Crazyflie Python Library
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Example script that makes the Crazyflie follow a wall

This examples uses the Flow and Multi-ranger decks to measure distances
in all directions and do wall following. Straight walls with corners
are advised to have in the test environment.
This is a python port of c-based app layer example from the Crazyflie-firmware
found here https://github.com/bitcraze/crazyflie-firmware/tree/master/examples/
demos/app_wall_following_demo

For the example to run the following hardware is needed:
 * Crazyflie 2.0
 * Crazyradio PA
 * Flow deck
 * Multiranger deck
"""
import logging
import math
import random
import time
from math import degrees
from math import radians

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

BOUNCE_SPEED = 0.9
BOUNCE_TURN_RATE = 1.5
WALL_THRESHOLD = 0.5
ANGLE_TOLERANCE = 0.15

STATE_FLY_FORWARD = "FLY_FORWARD"
STATE_ROTATING = "ROTATING"


def handle_range_measurement(range):
    if range is None:
        range = 999
    return range


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Only output errors from the logging framework
    logging.basicConfig(level=logging.ERROR)

    keep_flying = True
    bounce_state = STATE_FLY_FORWARD
    bounce_target_angle = math.pi
    bounce_angle_accumulated = 0.0
    bounce_prev_yaw = 0.0

    # Setup logging to get the yaw data
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
    lg_stab.add_variable('stabilizer.yaw', 'float')

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multiranger:
                with SyncLogger(scf, lg_stab) as logger:

                    while keep_flying:

                        velocity_x = 0.0
                        velocity_y = 0.0
                        yaw_rate = 0.0

                        log_entry = logger.next()
                        data = log_entry[1]
                        actual_yaw = data['stabilizer.yaw']
                        actual_yaw_rad = radians(actual_yaw)

                        front_range = handle_range_measurement(multiranger.front)
                        top_range = handle_range_measurement(multiranger.up)

                        if bounce_state == STATE_FLY_FORWARD:
                            velocity_x = BOUNCE_SPEED
                            if front_range < WALL_THRESHOLD:
                                bounce_state = STATE_ROTATING
                                bounce_target_angle = random.uniform(math.pi / 2, 3 * math.pi / 2)
                                bounce_angle_accumulated = 0.0
                                bounce_prev_yaw = actual_yaw_rad
                        elif bounce_state == STATE_ROTATING:
                            yaw_rate = BOUNCE_TURN_RATE
                            delta = abs(wrap_to_pi(actual_yaw_rad - bounce_prev_yaw))
                            bounce_angle_accumulated += delta
                            bounce_prev_yaw = actual_yaw_rad
                            if bounce_angle_accumulated >= bounce_target_angle:
                                bounce_state = STATE_FLY_FORWARD

                        print('velocity_x', velocity_x, 'yaw_rate', yaw_rate,
                              'state', bounce_state)

                        yaw_rate_deg = degrees(yaw_rate)

                        motion_commander.start_linear_motion(
                            velocity_x, velocity_y, 0, rate_yaw=yaw_rate_deg)

                        if top_range < 0.2:
                            keep_flying = False
