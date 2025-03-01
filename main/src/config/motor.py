
from typing import NamedTuple
from rev import SparkMaxConfig

class MotorConfig(NamedTuple):
    """Information to configure a motor on the RoboRIO"""
    id: int  # Motor ID on the RoboRIO
    inverted: bool  # Invert the motor
    motor_config: int # 0 for drive, 1 for turn
    # current_limit: int | None  = None  # Current limit in amps
    # open_ramp_rate: float | None = None  # Time in seconds to go from 0 to full throttle
    # closed_ramp_rate: float | None = None  # Time in seconds to go from 0 to full throttle