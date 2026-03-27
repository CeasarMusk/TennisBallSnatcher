class DroneStateMachine:

    takeoff_height = 1

    def __init__(self, fc):
        self.fc = fc
        self.state = "IDLE"
        print("Idle state\n")

    def update(self):
        if self.state == "IDLE":
            self.fc.set_stablize_mode()
            self.fc.arm()
            self.fc.set_guided_mode()
            print("guided state\n")
            self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            if not hasattr(self, "takeoff_sent"):
                print("attempting takeoff\n")
                self.fc.takeoff(takeoff_height)
                self.takeoff_sent = True

            alt = self.fc.get_altitude()
            if alt is not None and alt >= takeoff_height:
                self.state = "NAVIGATE"
                print("Takeoff success")
