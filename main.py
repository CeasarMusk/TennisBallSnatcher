from pymavlink import mavutil
from flightcontroller_state import FlightController
from mission_state import DroneStateMachine

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("connection made\n")
fc = FlightController(master)
sm = DroneStateMachine(fc)

while True:
    sm.update()
