class DroneStateMachine:
  def __init__(self, fc):
    self.fc = fc
    self.state = "IDLE"
  def update(self):
    if self.state == "IDLE":
      self.fc.set_guided_mode()
      self.fc.arm()
      self.state = "TAKEOFF"

    elif self.state == "TAKEOFF":
      self.fc.takeoff(3)
      self.state = "NAVIGATE"
