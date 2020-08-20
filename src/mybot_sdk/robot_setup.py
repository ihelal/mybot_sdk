#!/usr/bin/python3
import os
import yaml
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO

def get_robot_cfg():
    script_path = str(os.path.dirname(os.path.realpath(__file__)))
    config_path = script_path + "/mybot_config.yaml"
    with open(config_path, "r") as ymlfile:
        cfg = yaml.load(ymlfile)
    return cfg

class RobotSetup():
    def __init__(self):
        pass
    
    def get_robot_cfg(self):
        script_path = str(os.path.dirname(os.path.realpath(__file__)))
        config_path = script_path + "/mybot_config.yaml"
        with open(config_path, "r") as ymlfile:
            cfg = yaml.load(ymlfile)
        return cfg

class ServoBoard():
    def __init__(self):
        self.servo_board = ServoKit(channels=16)
    
    def get_board(self):
        return self.servo_board

class Brain():
    def __init__(self):
        self.robot_setup = RobotSetup().get_robot_cfg()
        GPIO.setwarnings(self.robot_setup["BoardConfig"]["IOWarn"])
        GPIO.setmode(GPIO.BCM)

    def initBaseMotors(self):
        drive = self.robot_setup["Actuators"]["Drive"]
        front_drive = drive["Front"]
        back_drive = drive["Back"]
        PWM_PIN = drive["PWM"]
        
        GPIO.setup(PWM_PIN, GPIO.OUT)

        GPIO.setup(front_drive["R"]["IN1"], GPIO.OUT) #problem
        GPIO.setup(front_drive["R"]["IN2"], GPIO.OUT)
        GPIO.setup(front_drive["L"]["IN1"], GPIO.OUT)
        GPIO.setup(front_drive["L"]["IN2"], GPIO.OUT)

        GPIO.setup(back_drive["R"]["IN1"], GPIO.OUT)
        GPIO.setup(back_drive["R"]["IN2"], GPIO.OUT) #problem
        GPIO.setup(back_drive["L"]["IN1"], GPIO.OUT)
        GPIO.setup(back_drive["L"]["IN2"], GPIO.OUT)

        pwm = GPIO.PWM(PWM_PIN, 100) # Connected to PWMA
        pwm.start(0)

        # Set all motors off
        GPIO.output(front_drive["R"]["IN1"], False)
        GPIO.output(front_drive["R"]["IN2"], False)
        GPIO.output(front_drive["L"]["IN1"], False)
        GPIO.output(front_drive["L"]["IN2"], False)

        GPIO.output(back_drive["R"]["IN1"], False)
        GPIO.output(back_drive["R"]["IN2"], False)
        GPIO.output(back_drive["L"]["IN1"], False)
        GPIO.output(back_drive["L"]["IN2"], False)

        return pwm,front_drive,back_drive

    def initIR(self):
        LineFollower = self.robot_setup["Sensors"]["LineFollower"]
        self.LINE_LEFT = LineFollower["L"]
        self.LINE_CENTER = LineFollower["C"]
        self.LINE_RIGHT = LineFollower["R"]

        GPIO.setup(self.LINE_LEFT, GPIO.IN)
        GPIO.setup(self.LINE_CENTER, GPIO.IN)
        GPIO.setup(self.LINE_RIGHT, GPIO.IN)

        return self.LINE_LEFT,self.LINE_CENTER,self.LINE_RIGHT

    def get_input_status(self,pin):
        status = GPIO.input(pin)
        return status

    def get_robot_compass(self):
        compass = self.robot_setup["Compass"]
        return compass

    def get_mecanum_setup(self):
        mecanum = self.robot_setup["Mecanum"]
        return mecanum

class Motor():
    def __init__(self):
        self.pwm,self.front_drive,self.back_drive = Brain().initBaseMotors()
        self.robotCompass = Brain().get_robot_compass()

    def moveMotor(self,motor,speed=10,direction=1):
        self.setSpeed(speed)
        H1,H2 = self.setMotorDirection(direction)
        P1,P2 = self.selectMotor(motor)
        GPIO.output(P1, H1)
        GPIO.output(P2, H2)
    
    def setMotorDirection(self,direction=1):
        if direction == 1: #CCW
            H1 = True
            H2 = False
            return H1,H2
        elif direction == -1: #CW
            H1 = False
            H2 = True
            return H1,H2
        elif direction == 0: #Not moving
            H1 = False
            H2 = False
            return H1,H2

    def selectMotor(self,motor):
        if motor == 0: #FL
            P1 = self.front_drive["L"]["IN1"]
            P2 = self.front_drive["L"]["IN2"]
            return P1,P2
        elif motor == 1: #BL
            P1 = self.back_drive["L"]["IN1"]
            P2 = self.back_drive["L"]["IN2"]
            return P1,P2
        elif motor == 2: #BR
            P1 = self.back_drive["R"]["IN1"]
            P2 = self.back_drive["R"]["IN2"]
            return P1,P2
        elif motor == 3: #FR
            P1 = self.front_drive["R"]["IN1"]
            P2 = self.front_drive["R"]["IN2"]
            return P1,P2
    def stopMotor(self,motor):
        P1,P2 = self.selectMotor(motor)
        GPIO.output(P1, False)
        GPIO.output(P2, False)

    def setSpeed(self,speed):
        self.pwm.ChangeDutyCycle(speed)
