import argparse
import time
from enum import Enum

import numpy as np
import math

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        
        # Group of parameters
        self.WP_TOLERANCE = 0.2  # How close should quad be to consider WP reached
        self.TAKEOFF_ALTITUDE = 3.0  # Altitude to reach while taking off
        self.HEADING_ALONG_ROUTE = True  # If quad should adjust yaw along route segments
        self.STABILIZE_AT_WP = True  # If quad should hover with low velocity at WP before starting next WP
        self.STABILIZE_AT_WP_SPEED_TOLERANCE = 0.5  # Min abs(velocity) value to consider that quad is stabilized

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """

        # For simplicity
        lp = self.local_position.copy()
        lp[2] = -lp[2]

        # Wait until quad reaches the 95% of takeoff_altitude and start waypoint mission
        if self.flight_state == States.TAKEOFF:
            if lp[2] > .95*self.target_position[2]:
                self.start_mission()
        # Wait until quad reaches WP and switches to next WP. If none available, go landing
        elif self.flight_state == States.WAYPOINT:
            dst = np.linalg.norm(lp - self.target_position)
            spd = np.linalg.norm(self.local_velocity)
            print('dst = {0:.2f}m'.format(dst))
            if dst < self.WP_TOLERANCE and (not self.STABILIZE_AT_WP or spd < self.STABILIZE_AT_WP_SPEED_TOLERANCE):
                if len(self.all_waypoints) == 0:
                    if spd < 0.5:  # wait until quad stabilizes its velocity
                        self.landing_transition()
                else:
                    self.target_position = self.all_waypoints.pop(0)
                    self.waypoint_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1 and abs(self.local_position[2]) < 0.02:
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return

        if self.flight_state == States.MANUAL: self.arming_transition()
        elif self.flight_state == States.ARMING: self.takeoff_transition()
        elif self.flight_state == States.DISARMING: self.manual_transition()

    def start_mission(self):
        """
        Retrieves waypoints and initiates waypoint_transition()
        """
        self.all_waypoints = self.calculate_box()
        self.target_position = self.all_waypoints.pop(0)
        self.waypoint_transition();

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        wps = [
            (0, 0, 6),
            (10, 0, 6),
            (10, 10, 6),
            (0, 10, 6)
        ]
        return wps + [wps[0]]

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        if not self.armed:  # no need to arm if the quad was already armed
            self.arm()
            self.set_home_position(*self.global_position)
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        if self.local_position[2] < self.TAKEOFF_ALTITUDE:  # takeoff only if the altitude is lower, then takeof_altitude
            self.target_position[2] = self.TAKEOFF_ALTITUDE
            self.takeoff(self.TAKEOFF_ALTITUDE)
            self.flight_state = States.TAKEOFF
        else:  # otherwise simply start mission
            self.start_mission()

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        lp, tp = self.local_position, self.target_position
        heading = math.atan2(tp[1]-lp[1], tp[0]-lp[0]) if self.HEADING_ALONG_ROUTE else 0
        self.cmd_position(*tp, heading)
        print('> Waypoint: {0}'.format(tp))  # debugging
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
