import wpilib
import wpilib.drive
import wpilib.interfaces
from enum import Enum, auto
import wpimath


class DeadzoneMode(Enum):

    CUTOFF = auto()
    """This will cut off any value"""

    SCALE = auto()
    """
    ! Not yet implemented

    This will move the scale from joystick_zero=0 to joystick_limit=1 to deadzone=0 and joystick_limit=1
    """


class DriveTrain:
    deadzone: float = 0
    deadzone_twist: float = 0.2
    deadzone_mode: DeadzoneMode = DeadzoneMode.CUTOFF
    speedMultiplier: float = 1
    twistMultiplier: float = 1
    magnitudeDeadzone: float = 0.2
    

    def __init__(self, leftFront: wpilib.interfaces.MotorController, leftRear: wpilib.interfaces.MotorController, rightFront: wpilib.interfaces.MotorController, rightRear: wpilib.interfaces.MotorController) -> None:
        self.leftFront = leftFront
        self.leftRear = leftRear
        self.rightFront = rightFront
        self.rightRear = rightRear
        self.stick = wpilib.Joystick(0)
        self.realX = 0
        self.realY = 0
        self.realZ = 0

    def constrainJoystick(self, Joystick: wpilib.Joystick):
        """
        Constrains joystick using deadzone & deadzone_twist values & applies speed multiplier

        anything below the deadzone/deadzone_twist value will be cut off(set to 0)
        """
        mag = Joystick.getMagnitude()
        angle = Joystick.getDirectionDegrees()
        rotate = Joystick.getTwist()
        if mag < self.magnitudeDeadzone:  # implement based on self.deadzone_mode
            mag = 0
        if abs(rotate) < self.deadzone_twist:  # absolute value b/c rotate goes from -1 to 1
            rotate = 0

        mag *= self.speedMultiplier
        rotate *= self.twistMultiplier
        return [mag, angle, rotate]

    def setDeadzone(self, deadzone_move: float, deadzone_twist: float, deadzone_mode: DeadzoneMode = DeadzoneMode.CUTOFF):
        self.deadzone = deadzone_move
        self.deadzone_twist = deadzone_twist
        self.deadzone_mode = deadzone_mode

    def drive(self, Joystick: wpilib.Joystick) -> None:
        """
        DO NOT REPLACE!

        override moveRobot instead
        """
        self.moveRobot(*self.constrainJoystick(Joystick))

    #def moveRobot(self, speed: float, direction: float, twist: float):
        """
        Drive the robot in a direction at a speed for a duration

        :param speed: the speed of the robot[0, 1]

        :param direction: angle to drive at from [-180, 180]

        Angles are measured clockwise from the positive X axis. The robot's speed is independent from its angle or rotation rate.

        :param twist: the speed of the robot in the z(rotational) axis[-1, 1]
        """
        #raise ValueError("THIS SHOULD BE REPLACED!")

    def rightInverted(self, isInverted: bool) -> None:
        self.rightFront.setInverted(isInverted)
        self.rightRear.setInverted(isInverted)

    def leftInverted(self, isInverted: bool) -> None:
        self.leftFront.setInverted(isInverted)
        self.leftRear.setInverted(isInverted)

    def motorTest(self, timer: wpilib.Timer) -> None:
        # Test each motor one by one
        # FR FL RR RL
        duration = 3
        speed = 0.4

        self.rightFront.set(speed)
        if timer.get() > duration*4:
            self.leftRear.stopMotor()
        elif timer.get() > duration*3:
            self.rightRear.stopMotor()
            self.leftRear.set(speed)
        elif timer.get() > duration*2:
            self.leftFront.stopMotor()
            self.rightRear.set(speed)
        elif timer.get() > duration*1:
            self.rightFront.stopMotor()
            self.leftFront.set(speed)
    




class MecanumDrive(DriveTrain):
    def __init__(self, leftFront: wpilib.interfaces.MotorController, leftRear: wpilib.interfaces.MotorController, rightFront: wpilib.interfaces.MotorController, rightRear: wpilib.interfaces.MotorController,gyro: wpilib.ADXRS450_Gyro) -> None:
        # run the parent's __init__ function
        super().__init__(leftFront, leftRear, rightFront, rightRear)
        self.MecanumDrive = wpilib.drive.MecanumDrive(
            self.leftRear, self.leftFront, self.rightRear, self.rightFront)  # create a mecanum drive object
        self.gyro = gyro

    def moveRobot(self, speed: float, direction: float, twist: float):
        # self.stickInputY = self.stick.getY()
        # self.stickInputX = self.stick.getX()
        # self.stickInputZ = self.stick.getZ()
        

        # if (abs(self.stickInputY) < 0.2):
        #     self.realY = 0
        # else:
        #     self.realY = self.stickInputY
        # if (abs(self.stickInputX) < 0.2):
        #     self.realX = 0
        # else:
        #     self.realX = self.stickInputX
        # if (abs(self.stickInputZ) < 0.2):
        #     self.realZ = 0
        # else: 
        #     self.realZ = self.stickInputZ
        
        # self.realY = (self.realY * self.speedMultiplier)
        # self.realX = (self.realX * self.speedMultiplier)
        # self.realZ = (self.realZ * self.speedMultiplier)

        #if self.timer.get() < 15:
            #self.realY = speed 
            #self.realX = direction 
            #self.realZ = twist
        magnitude = self.stick.getMagnitude()
        twist = -self.stick.getZ()
        self.stickInputY = self.stick.getY()
        self.stickInputX = self.stick.getX()
        self.stickInputZ = self.stick.getZ()

        if magnitude < DriveTrain.magnitudeDeadzone:
            magnitude = 0
        if abs(twist) < DriveTrain.deadzone_twist:
            twist = 0
        if self.stick.getRawButton(2) > 0:
            magnitude *= 0.25
        else:
            magnitude = magnitude
        direction = self.stick.getDirectionDegrees()+90
        direction += -self.gyro.getAngle()
        # self.MecanumDrive.driveCartesian(-self.realX, -self.realY, -self.realZ)
        if self.stick.getTrigger() > 0:
            self.MecanumDrive.drivePolar(magnitude,wpimath.geometry.Rotation2d.fromDegrees(direction), twist)
        else:
            self.MecanumDrive.driveCartesian(-self.stickInputX, -self.stickInputY, -self.stickInputZ)

        #self.MecanumDrive.driveCartesian(speed, direction, twist)
        #self.MecaumDrive.driveCartesian(speed,direction,twist)     
