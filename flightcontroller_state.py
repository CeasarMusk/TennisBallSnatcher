from pymavlink import mavutil
import time

class FlightController:
    def __init__(self, connection):
        self.master = connection

    def arm(self):
        self.master.arducopter_arm()
        self.master.motors_armed_wait()

    def takeoff(self, altitude):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude
        )

    def land(self):
        self.master.set_mode_apm("LAND")
