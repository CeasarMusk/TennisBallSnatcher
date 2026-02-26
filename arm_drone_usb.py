from pymavlink import mavutil

def connect_pixhawk():
    master = mavutil.mavlink_connection('/dev/ttyACM1', baud=921600)
    master.wait_heartbeat()
    print("Connected to Pixhawk on /dev/ttyACM1")
    return master

if __name__ == "__main__":
    master = connect_pixhawk()
