# import wpilib
# import wpilib.drive
# import wpilib.interfaces
# from enum import Enum, auto
# import wpimath
# from dataclasses import dataclass

# # I DON'T KNOW IF NEO IS THE CORRECT MOTOR TYPE, AND THIS IS POSSIBLY AN ALTERNATIVE TO IMPORT REV THT CHAT GPT MADE
# #from rev import CANSparkMax, Neo

# class DeadzoneMode(Enum):

#     CUTOFF = auto()
#     """This will cut off any value"""

#     SCALE = auto()
#     """
#     ! Not yet implemented

#     This will move the scale from joystick_zero=0 to joystick_limit=1 to deadzone=0 and joystick_limit=1
#     """


# class DriveTrain:
#     deadzone: float = 0
#     deadzone_twist: float = 0.2
#     deadzone_mode: DeadzoneMode = DeadzoneMode.CUTOFF
#     speedMultiplier: float = 1
#     twistMultiplier: float = 1
#     magnitudeDeadzone: float = 0.2
    

#     def __init__(self, leftFront1: wpilib.interfaces.MotorController, leftRear1: wpilib.interfaces.MotorController, rightFront1: wpilib.interfaces.MotorController, rightRear1: wpilib.interfaces.MotorController, leftFront2: wpilib.interfaces.MotorController, leftRear2: wpilib.interfaces.MotorController, rightFront2: wpilib.interfaces.MotorController, rightRear2: wpilib.interfaces.MotorController) -> None:
#         self.leftFront1 = leftFront1
#         self.leftFront2 = leftFront2
#         self.leftRear1 = leftRear1
#         self.leftRear2 = leftRear2
#         self.rightFront1 = rightFront1
#         self.rightFront2 = rightFront2
#         self.rightRear1 = rightRear1
#         self.rightRear2 = rightRear1
#         self.stick = wpilib.Joystick(0)
#         self.realX = 0
#         self.realY = 0
#         self.realZ = 0

#     def constrainJoystick(self, Joystick: wpilib.Joystick):
#         """
#         Constrains joystick using deadzone & deadzone_twist values & applies speed multiplier

#         anything below the deadzone/deadzone_twist value will be cut off(set to 0)
#         """
#         mag = Joystick.getMagnitude()
#         angle = Joystick.getDirectionDegrees()
#         rotate = Joystick.getTwist()
#         if mag < self.magnitudeDeadzone:  # implement based on self.deadzone_mode
#             mag = 0
#         if abs(rotate) < self.deadzone_twist:  # absolute value b/c rotate goes from -1 to 1
#             rotate = 0

#         mag *= self.speedMultiplier
#         rotate *= self.twistMultiplier
#         return [mag, angle, rotate]

#     def setDeadzone(self, deadzone_move: float, deadzone_twist: float, deadzone_mode: DeadzoneMode = DeadzoneMode.CUTOFF):
#         self.deadzone = deadzone_move
#         self.deadzone_twist = deadzone_twist
#         self.deadzone_mode = deadzone_mode

#     def drive(self, Joystick: wpilib.Joystick) -> None:
#         """
#         DO NOT REPLACE!

#         override moveRobot instead
#         """
#         self.moveRobot(*self.constrainJoystick(Joystick))

#     #def moveRobot(self, speed: float, direction: float, twist: float):
#         """
#         Drive the robot in a direction at a speed for a duration

#         :param speed: the speed of the robot[0, 1]

#         :param direction: angle to drive at from [-180, 180]

#         Angles are measured clockwise from the positive X axis. The robot's speed is independent from its angle or rotation rate.

#         :param twist: the speed of the robot in the z(rotational) axis[-1, 1]
#         """
#         #raise ValueError("THIS SHOULD BE REPLACED!")

#     def rightInverted(self, isInverted: bool) -> None:
#         self.rightFront1.setInverted(isInverted)
#         self.rightRear1.setInverted(isInverted)
#         self.rightFront2.setInverted(isInverted)
#         self.rightRear2.setInverted(isInverted)

#     def leftInverted(self, isInverted: bool) -> None:
#         self.leftFront1.setInverted(isInverted)
#         self.leftRear2.setInverted(isInverted)
#         self.leftFront1.setInverted(isInverted)
#         self.leftRear2.setInverted(isInverted)


#     def motorTest(self, timer: wpilib.Timer) -> None:
#         # Test each motor one by one
#         # FR FL RR RL
#         duration = 3
#         speed = 0.4

#         self.rightFront1.set(speed)
#         if timer.get() > duration*4:
#             self.leftRear1.stopMotor()
#         elif timer.get() > duration*3:
#             self.rightRear1.stopMotor()
#             self.leftRear1.set(speed)
#         elif timer.get() > duration*2:
#             self.leftFront1.stopMotor()
#             self.rightRear1.set(speed)
#         elif timer.get() > duration*1:
#             self.rightFront1.stopMotor()
#             self.leftFront1.set(speed)
#         self.rightFront2.set(speed)
#         if timer.get() > duration*4:
#             self.leftRear2.stopMotor()
#         elif timer.get() > duration*3:
#             self.rightRear2.stopMotor()
#             self.leftRear2.set(speed)
#         elif timer.get() > duration*2:
#             self.leftFront2.stopMotor()
#             self.rightRear2.set(speed)
#         elif timer.get() > duration*1:
#             self.rightFront2.stopMotor()
#             self.leftFront2.set(speed)
    

# @dataclass
# class SparkMaxSwerveNode(SwerveNode):
#     m_move: SparkMax
#     m_turn: SparkMax
#     encoder: CANCoder
#     absolute_encoder_zeroed_pos: radians = 0
#     name: str = "DefaultNode"

#     def init(self):
#         super().init()
#         self.m_move.init()
#         self.m_turn.init()

#     def initial_zero(self):
#         current_pos_rad = (
#             math.radians(self.encoder.getAbsolutePosition())
#             - self.absolute_encoder_zeroed_pos
#         )

#         self.m_turn.set_sensor_position(
#             current_pos_rad * constants.drivetrain_turn_gear_ratio / (2 * math.pi)
#         )

#         self.m_move.set_sensor_position(0)
#         self.m_move.set_target_position(0)

#     def zero(self):
#         current_angle = self.get_current_motor_angle()

#         current_pos_rad = (
#             math.radians(self.encoder.getAbsolutePosition())
#             - self.absolute_encoder_zeroed_pos
#         )

#         self.m_turn.set_sensor_position(
#             current_pos_rad * constants.drivetrain_turn_gear_ratio / (2 * math.pi)
#         )

#         self.set_motor_angle(current_angle)

#     def raw_output(self, power):
#         self.m_move.set_raw_output(power)

#     def set_motor_angle(self, pos: radians):
#         self.m_turn.set_target_position(
#             (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio
#         )

#     def direct_set_motor_angle(self, pos: radians):
#         self.m_turn.set_target_position(
#             (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio
#         )

#     def get_current_motor_angle(self) -> radians:
#         return (
#             (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
#             * 2
#             * math.pi
#         )

#     def set_motor_velocity(self, vel: meters_per_second):
#         self.m_move.set_target_velocity(vel * constants.drivetrain_move_gear_ratio)

#     def get_motor_velocity(self) -> radians_per_second:
#         return (
#             self.m_move.get_sensor_velocity()
#             / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
#         )

#     def get_drive_motor_traveled_distance(self) -> meters:
#         sensor_position = -1 * self.m_move.get_sensor_position()

#         return (
#             sensor_position
#             / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
#         )

#     def get_turn_motor_angle(self) -> radians:
#         return (
#             (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
#             * 2
#             * math.pi
#         )


# class Drivetrain(SwerveDrivetrain):
#     n_front_left1 = SparkMaxSwerveNode(
#         SparkMax(16, config=MOVE_CONFIG),
#         SparkMax(15, config=TURN_CONFIG),
#         CANCoder(24),
#         absolute_encoder_zeroed_pos=math.radians(174.638),
#         name="n_front_left",
#     )
#     n_front_right1 = SparkMaxSwerveNode(
#         SparkMax(14, config=MOVE_CONFIG),
#         SparkMax(13, config=TURN_CONFIG),
#         CANCoder(23),
#         absolute_encoder_zeroed_pos=math.radians(282.304),
#         name="n_front_right",
#     )
#     n_back_left1 = SparkMaxSwerveNode(
#         SparkMax(3, config=MOVE_CONFIG),
#         SparkMax(4, config=TURN_CONFIG),
#         CANCoder(21),
#         absolute_encoder_zeroed_pos=math.radians(313.769),
#         name="n_back_left",
#     )
#     n_back_right1 = SparkMaxSwerveNode(
#         SparkMax(5, config=MOVE_CONFIG),
#         SparkMax(6, config=TURN_CONFIG),
#         CANCoder(22),
#         absolute_encoder_zeroed_pos=math.radians(136.58),
#         name="n_back_right",
#     )
#     n_front_left2 = SparkMaxSwerveNode(
#         SparkMax(16, config=MOVE_CONFIG),
#         SparkMax(15, config=TURN_CONFIG),
#         CANCoder(24),
#         absolute_encoder_zeroed_pos=math.radians(174.638),
#         name="n_front_left",
#     )
#     n_front_right2 = SparkMaxSwerveNode(
#         SparkMax(14, config=MOVE_CONFIG),
#         SparkMax(13, config=TURN_CONFIG),
#         CANCoder(23),
#         absolute_encoder_zeroed_pos=math.radians(282.304),
#         name="n_front_right",
#     )
#     n_back_left2 = SparkMaxSwerveNode(
#         SparkMax(3, config=MOVE_CONFIG),
#         SparkMax(4, config=TURN_CONFIG),
#         CANCoder(21),
#         absolute_encoder_zeroed_pos=math.radians(313.769),
#         name="n_back_left",
#     )
#     n_back_right2 = SparkMaxSwerveNode(
#         SparkMax(5, config=MOVE_CONFIG),
#         SparkMax(6, config=TURN_CONFIG),
#         CANCoder(22),
#         absolute_encoder_zeroed_pos=math.radians(136.58),
#         name="n_back_right",
#     )

#     gyro: PigeonIMUGyro_Wrapper = PigeonIMUGyro_Wrapper(20)
#     axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
#     axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
#     axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
#     track_width: meters = constants.track_width
#     max_vel: meters_per_second = constants.drivetrain_max_vel
#     max_target_accel: meters_per_second_squared = constants.drivetrain_max_target_accel
#     max_angular_vel: radians_per_second = constants.drivetrain_max_angular_vel
#     deadzone_velocity: meters_per_second = 0.02
#     deadzone_angular_velocity: radians_per_second = math.radians(5)
#     start_angle: degrees = 0
#     start_pose: Pose2d = Pose2d(
#         0,
#         0,
#         math.radians(start_angle),
#     )
#     gyro_start_angle: radians = math.radians(start_angle)
#     gyro_offset: radians = math.radians(0)

#     def x_mode(self):
#         self.n_front_left.set_motor_angle(math.radians(-45))
#         self.n_front_right.set_motor_angle(math.radians(45))
#         self.n_back_left.set_motor_angle(math.radians(45))
#         self.n_back_right.set_motor_angle(math.radians(-45))



# class MecanumDrive(DriveTrain):
#     def __init__(self, leftFront: wpilib.interfaces.MotorController, leftRear: wpilib.interfaces.MotorController, rightFront: wpilib.interfaces.MotorController, rightRear: wpilib.interfaces.MotorController,gyro: wpilib.ADXRS450_Gyro) -> None:
#         # run the parent's __init__ function
#         super().__init__(leftFront, leftRear, rightFront, rightRear)
#         self.MecanumDrive = wpilib.drive.MecanumDrive(
#             self.leftRear, self.leftFront, self.rightRear, self.rightFront)  # create a mecanum drive object
#         self.gyro = gyro

#     def moveRobot(self, speed: float, direction: float, twist: float):
#         # self.stickInputY = self.stick.getY()
#         # self.stickInputX = self.stick.getX()
#         # self.stickInputZ = self.stick.getZ()
        

#         # if (abs(self.stickInputY) < 0.2):
#         #     self.realY = 0
#         # else:
#         #     self.realY = self.stickInputY
#         # if (abs(self.stickInputX) < 0.2):
#         #     self.realX = 0
#         # else:
#         #     self.realX = self.stickInputX
#         # if (abs(self.stickInputZ) < 0.2):
#         #     self.realZ = 0
#         # else: 
#         #     self.realZ = self.stickInputZ
        
#         # self.realY = (self.realY * self.speedMultiplier)
#         # self.realX = (self.realX * self.speedMultiplier)
#         # self.realZ = (self.realZ * self.speedMultiplier)

#         #if self.timer.get() < 15:
#             #self.realY = speed 
#             #self.realX = direction 
#             #self.realZ = twist
#         magnitude = self.stick.getMagnitude()
#         twist = -self.stick.getZ()
#         self.stickInputY = self.stick.getY()
#         self.stickInputX = self.stick.getX()
#         self.stickInputZ = self.stick.getZ()

#         if magnitude < DriveTrain.magnitudeDeadzone:
#             magnitude = 0
#         if abs(twist) < DriveTrain.deadzone_twist:
#             twist = 0
#         if self.stick.getRawButton(2) > 0:
#             magnitude *= 0.25
#         else:
#             magnitude = magnitude
#         direction = self.stick.getDirectionDegrees()+90
#         direction += -self.gyro.getAngle()
#         # self.MecanumDrive.driveCartesian(-self.realX, -self.realY, -self.realZ)
#         if self.stick.getTrigger() > 0:
#             self.MecanumDrive.drivePolar(magnitude,wpimath.geometry.Rotation2d.fromDegrees(direction), twist)
#         else:
#             self.MecanumDrive.driveCartesian(-self.stickInputX, -self.stickInputY, -self.stickInputZ)

#         #self.MecanumDrive.driveCartesian(speed, direction, twist)
#         #self.MecaumDrive.driveCartesian(speed,direction,twist)     