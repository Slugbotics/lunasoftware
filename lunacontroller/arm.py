from gpiozero import AngularServo

class Arm:
    def __init__(self):
        self.angle = 90
        self.servo = AngularServo(pin=17, min_angle=0, max_angle=180)
    
    def set_angle(self, angle):
        self.angle = angle
        self.servo.angle = angle

armInstance = Arm()