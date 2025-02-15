#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

from enum import Enum
import wpilib
import wpilib.drive

# our code imports
import drivetrain
import pneumatics
import autonomous
import winch
import ports
# from vision import cameraLaunch
import ports


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        # cameraLaunch()
        self.gyro = wpilib.ADXRS450_Gyro()
        
        wpilib.SmartDashboard.putNumber('Gyro Angle', self.gyro.getAngle())
        #print('this port' + str(self.gyro.getPort()))
        #wpilib.SmartDashboard.putNumber('GyroAngle', self.gyro.getAngle())

        
        # self.solenoidDump = wpilib.DoubleSolenoid(
        #     wpilib.PneumaticsModuleType.CTREPCM, 1, 0)
        # self.solenoid2 = wpilib.DoubleSolenoid(
        #     wpilib.PneumaticsModuleType.CTREPCM, 3, 2)
        # self.solenoid3 = wpilib.DoubleSolenoid(
        #     wpilib.PneumaticsModuleType.CTREPCM, 5, 4)

        self.solenoidExtend = pneumatics.DoubleSolenoid(
            2,3)
        self.solenoidClamp = pneumatics.DoubleSolenoid(
            1,0)
        #self.solenoidClimb2 = pneumatics.DoubleSolenoid(
            #*ports.PneumaticPorts.CLIMB2)

        self.leftFront = wpilib.Talon(ports.MotorPorts.LEFT_FRONT)
        self.leftRear = wpilib.Talon(ports.MotorPorts.LEFT_REAR)
        self.rightFront = wpilib.Talon(ports.MotorPorts.RIGHT_FRONT)
        self.rightRear = wpilib.Talon(ports.MotorPorts.RIGHT_REAR)

        self.safeLock = 0

        self.leftWinchMotor = wpilib.Spark(ports.MotorPorts.LEFT_WINCH)
        self.rightWinchMotor = wpilib.Talon(ports.MotorPorts.RIGHT_WINCH)
        self.rightWinchMotor.setInverted(True)

        self.leftWinch = winch.Winch(self.leftWinchMotor)
        self.rightWinch = winch.Winch(self.rightWinchMotor)

        self.windUp = wpilib.Talon(ports.MotorPorts.WIND_UP)
        self.shoot = wpilib.Talon(ports.MotorPorts.SHOOT)
        self.intakeSpin = wpilib.Talon(ports.MotorPorts.INTAKE_SPIN)
        self.intakeArm = wpilib.Talon(ports.MotorPorts.INTAKE_ARM)


        # self.drive = wpilib.drive.MecanumDrive(self.leftFront, self.leftRear, self.rightFront, self.rightRear)

        self.drivetrain = drivetrain.MecanumDrive(
            self.leftFront, self.leftRear, self.rightFront, self.rightRear,self.gyro)
        self.drivetrain.rightInverted(False)
        self.drivetrain.leftInverted(True)
        self.drivetrain.setDeadzone(0.5, 0.5)
        self.drivetrain.speedMultiplier = 1
        self.drivetrain.twistMultiplier = 1

        

        # self.rightFront.setInverted(True)
        # self.rightRear.setInverted(True)
        # self.leftFront.setInverted(True)
        # self.leftRear.setInverted(True) I would keep this commented out unless it drives in the wrong direction then you can revise.

        self.stick = wpilib.Joystick(ports.JoystickPorts.JOY)

        self.timer = wpilib.Timer()

    def teleopInit(self) -> None:
        self.timer.reset()
        self.timer.start()
        
    def teleopPeriodic(self):
        """This function is called periodically during operator control."""

        # Toggle pistons on button 3
        wpilib.SmartDashboard.putNumber('Gyro Angle', self.gyro.getAngle())
        #if self.safeLock == 0 and self.stick.getRawButtonPressed(ports.JoystickButtons.EXTENDTOGGLE):
        #    self.solenoidExtend.toggle()#

        #if self.stick.getRawButtonPressed(ports.JoystickButtons.CLAMPTOGGLE):
        #    self.solenoidClamp.toggle()

        #if self.solenoidClamp.getState() == False:
        #    self.safeLock = 1
        #else:
        #    self.safeLock = 0

        if self.stick.getRawButtonPressed(11):
            if self.gyro.getAngle() > 1:
                self.leftFront.set(.5)

        if self.stick.getRawButton(6) > 0: #winch go up 
            self.rightWinchMotor.set(1)
            self.leftWinchMotor.set(1)
        elif self.stick.getRawButton(4) > 0: #winch go down
            self.rightWinchMotor.set(-1)
            self.leftWinchMotor.set(-1)
        else: #winch stop
            self.rightWinchMotor.set(0)
            self.leftWinchMotor.set(0)
        """

        if self.stick.getRawButton(1) > 0: #Wind up for shooting
            self.windUp.set(1)
        else:
            self.windUp.set(0)

        if self.stick.getRawButton(2) > 0: #Shoot
            self.shoot.set(0.25) #may need tweaked
        else:
            self.shoot.set(0)

        if self.stick.getRawButton(4) > 0: #lower arm
            self.intakeArm.set(0.5) #may need inverted 
        elif self.stick.getRawButton(6) > 0: #raise arm
            self.intakeArm.set(-0.5) #may need inverted
        else:
            self.intakeArm.set(0)

        if self.stick.getRawButton(3) > 0: #lower arm
            self.intakeSpin.set(0.5) #speed needs tweaked 
        else:
            self.intakeSpin.set(0)

        """
            


        # Toggle speed multiplier on button 2
        if self.stick.getRawButtonPressed(ports.JoystickButtons.SPEEDMULTIPLIER):
            if self.drivetrain.speedMultiplier == 1:
                self.drivetrain.speedMultiplier = 0.5
            else:
                self.drivetrain.speedMultiplier = 1

        self.drivetrain.drive(self.stick)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()
    
    # autonomous.autonomousInit()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        if (self.timer.get() < 2.25):
            self.leftFront.set(-.5) #-1, 1, 1, -1 is FORWARD!!!!!!!!!!!!!!!!!!!! Port 1
            self.leftRear.set(.5) #1, -1, -1, 1 is BACKWARD!!!!!!!!!!!!!!!!!!!!! Port 0
            self.rightFront.set(.5)  #0, -1, 0, -1 OR 0, 1, 0, 1 is LEFT!!!!!!!! Port 3 
            self.rightRear.set(-.5)  #-1, 0, -1, 0 OR 1, 0, 1, 0 is RIGHT!!!!!!! Port 2
            #self.drivetrain.(self.realY, -self.realZ, -self.realX)
            #self.solenoidClamp.close()
            #self.solenoidExtend.open()
            #wpilib.SmartDashboard.putNumber('Gyro Angle', self.gyro.getAngle())
        elif self.timer.get() > 3 and self.timer.get() < 10:
            self.leftFront.set(-0.25) #-1, 1, 1, -1 is FORWARD!!!!!!!!!!!!!!!!!!!! Port 1
            self.leftRear.set(-0.25) #1, -1, -1, 1 is BACKWARD!!!!!!!!!!!!!!!!!!!!! Port 0
            self.rightFront.set(-0.25)  #0, -1, 0, -1 OR 0, 1, 0, 1 is LEFT!!!!!!!! Port 3 
            self.rightRear.set(-0.25)  #-1, 0, -1, 0 OR 1, 0, 1, 0 is RIGHT!!!!!!! Port 2
        else:
            self.drivetrain.moveRobot(0,0,0)
            self.leftFront.set(0)
            self.leftRear.set(0)
            self.rightFront.set(0)
            self.rightRear.set(0)             
'''            while (self.gyro.getAngle() < -5): #strafe  #THIS GOES ON LINE 107 NORMALLY; NOT USING PNEUMATICS, SO COMMENTED OUT
                self.leftFront.set(-0.3)
                self.leftRear.set(-0.3)
                self.rightFront.set(-0.3)
                self.rightRear.set(-0.3)
                wpilib.SmartDashboard.putNumber('Gyro Angle', self.gyro.getAngle())
            while (self.gyro.getAngle() > 5):
                self.leftFront.set(0.3)
                self.leftRear.set(0.3)
                self.rightFront.set(0.3)
                self.rightRear.set(0.3)
                wpilib.SmartDashboard.putNumber('Gyro Angle', self.gyro.getAngle())
'''
    


if __name__ == "__main__":
    wpilib.run(Robot)