import rev
import logging
import wpilib
import commands2

import hardware
from config import IntakeConfig


class Intake(commands2.Subsystem):
    intake_motor: rev.SparkMax
    intake_encoder: rev.SparkRelativeEncoder
    _intake_pid: rev.SparkClosedLoopController
    config: IntakeConfig
    _logger: logging.Logger

    @property
    def pid(self) -> rev.SparkClosedLoopController:
        return self._intake_pid

    def __init__(self, config: IntakeConfig, logger: logging.Logger):
        super().__init__()
        self.config = config
        self._logger = logger.getChild("Intake")
        self.intake_motor = rev.SparkMax(config.motor.id, rev.SparkMax.MotorType.kBrushless)
        hardware.init_motor(self.intake_motor, config.motor)
        self.intake_encoder = self.intake_motor.getEncoder()
        self._intake_pid = self.intake_motor.getPIDController()
        hardware.init_pid(self._intake_pid, self.config.pid, feedback_device=self.intake_encoder)

        self.intake_encoder.setPositionConversionFactor(1)
        self.intake_encoder.setVelocityConversionFactor(1)

    @property
    def intake_velocity(self) -> float:
        return self.intake_encoder.getVelocity()

    @intake_velocity.setter
    def intake_velocity(self, value: float):
        self._intake_pid.setReference(value, rev.SparkMax.ControlType.kVelocity)
        if value == 0:  #If we are stopping, reset the encoder
            self.intake_encoder.setPosition(0)

    @property
    def speed(self):
        return self.intake_motor.get()

    @speed.setter
    def speed(self, value: float):
        self.intake_motor.set(value)

    @property
    def voltage(self) -> float:
        return self.intake_motor.getBusVoltage()

    @voltage.setter
    def voltage(self, value: float):
        self.intake_motor.setVoltage(value)
