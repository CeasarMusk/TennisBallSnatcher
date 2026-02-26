from pymavlink import mavutil
import time
import math

def connect_sim():
    master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
    # master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    master.wait_heartbeat()
    print("Connected to simulation")
    return master


if __name__ == "__main__":
    master = connect_sim()
