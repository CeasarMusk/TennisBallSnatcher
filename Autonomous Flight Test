from pymavlink import mavutil
import time
import math

def get_position(master):
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    return msg.x, msg.y, msg.z



def generate_square(size):
    return [
        (0, 0, -2),
        (size, 0, -2),
        (size, size, -2),
        (0, size, -2),
    ]



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
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550') #master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
    master.wait_heartbeat()
    print("Connected to simulation")
    return master



def set_guided_and_arm(master):
    # 1. Wait for the M10 GPS to get a 3D Fix
    print("Checking GPS status...")
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg.fix_type >= 3: # 3 = 3D Fix, 4 = DGPS, etc.
            print(f"GPS Fix Ready! Satellites: {msg.satellites_visible}")
            break
        print(f"Waiting for GPS Fix... (Current fix type: {msg.fix_type})")
        time.sleep(1)
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
    wait_until_reached(master, 0, 0, -2)



def move_local(master, x, y, z, yaw):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000101111111000, #bit mask from position(bit 0-2), velocity(bit 3-5), accelration(bit 6-8), force(bit 9) yaw(bit 10), yaw rate(bit 11)
        x, y, z, #position
        0,0,0,   #velocity
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

    square = generate_square(size = 3)
    try:
     while True:
       for corner in square:
           target_x, target_y, target_z = corner
           move_local(master, target_x, target_y, target_z, 0)
           wait_until_reached(master, target_x, target_y, target_z)

    except KeyboardInterrupt:
        print("Loop Stopped by User.")
        land(master)
