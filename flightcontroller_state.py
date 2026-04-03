from pymavlink import mavutil
import time


class FlightController:

    def __init__(self, connection):
        self.master = connection

    def set_stablize_mode(self):
        self.master.set_mode("STABILIZE")
        
    def set_guided_mode(self):
        self.master.set_mode_apm("GUIDED")

    def arm(self):
        self.master.arducopter_arm()
        print("attemping to arm")
        self.master.motors_armed_wait()
        print("armed")
    def disarm(self):
        print("Attempting to disarm...")
        self.master.arducopt
    # Force the motors to stop immediately
    # 0 = disarm, 1 = arm
    # 21196 is the "force" parameter to ensure it happens even if not in a safe state
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        193, # MAV_CMD_COMPONENT_ARM_DISARM
        0,
        0, 0, 0, 0, 0, 0, 0 # Parameters
        )
    # Wait for the confirmation that motors have stopped
        self.master.motors_disarmed_wait()
        print("Disarmed successfully")

    

    def takeoff(self, altitude):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude
        )

    def get_altitude(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)

        if msg is None:
            return None

        # convert mm → meters
        return msg.relative_alt / 1000.0

    def land(self):
        self.master.set_mode_apm("LAND")
