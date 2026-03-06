from pymavlink import mavutil
import time
import math

def get_position(master):
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    return msg.x, msg.y, msg.z
    
def wait_until_reached(master, target_x, target_y, target_z, tolerance=0.2):
    while True:
        x, y, z = get_position(master)

        distance = math.sqrt(
            (target_x - x)**2 +
            (target_y - y)**2 +
            (target_z - z)**2
        )

        print(f"Distance to target: {distance:.2f} m")

        if distance < tolerance:
            print("Target reached!")
            break

        time.sleep(0.5)

def connect_sim():
    master = mavutil.mavlink_connection('/dev/serial0', baud=921600 ) #master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    master.wait_heartbeat()
    print("Connected to simulation")
    return master

def set_guided_and_arm(master):
    mode_id = master.mode_mapping()['GUIDED']
    master.set_mode(mode_id)
    time.sleep(2)

    master.arducopter_arm()
    master.motors_armed_wait()
    print("Armed")

def takeoff(master, altitude):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0,0,0,0, 0,0, altitude
    )
    print("Taking off...")
    time.sleep(8)

def move_local(master, vx, vy, vz, yaw):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000101111000111, #bit mask from position(bit 0-2), velocity(bit 3-5), accelration(bit 6-8), force(bit 9) yaw(bit 10), yaw rate(bit 11)
        0, 0, 0, #position
        vx,vy,vz,   #velocity
        0,0,0,   #acceleration
        yaw,0      #yaw, yaw rate
    )

def land(master):
    master.set_mode(master.mode_mapping()['LAND'])
    print("Landing...")
    time.sleep(10)

    #remember NED is North East Down 

if __name__ == "__main__":
    master = connect_sim()
    set_guided_and_arm(master)
    takeoff(master, 2)

    # -------------------------
    #  Move forward 0.2 m/s for 2 seconds
    print("Moving forward 0.2 m/s")
    start = time.time()
    while time.time() - start < 2:
        move_local(master, 0.2, 0, 0, 0)  # vx=0.2 forward, yaw=0 keeps current heading
        time.sleep(0.1)
    move_local(master, 0, 0, 0, 0)  # stop

    # -------------------------
    #  Rotate 180° in place
    print("Rotating 180°")
    start = time.time()
    while time.time() - start < 5:
        move_local(master, 0, 0, 0, math.pi)  # vx=0, rotate to 180°
        time.sleep(0.1)
    move_local(master, 0, 0, 0, math.pi)  # hold final yaw

    # -------------------------
    #  Move “back” to start (forward in BODY frame now points back)
    print("Moving back 0.2 m/s")
    start = time.time()
    while time.time() - start < 2:
        move_local(master, 0.2, 0, 0, 0)  # vx=0.2, yaw=0 keeps orientation constant
        time.sleep(0.1)
    move_local(master, 0, 0, 0, 0)  # stop

    # -------------------------
    #  Land
    land(master)
