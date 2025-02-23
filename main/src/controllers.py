from typing import NamedTuple
from config.driver_controls import ControllerType
import wpilib

# was ControllerKey
# A dictionary of controllers
controllers = dict[ControllerType, wpilib.Joystick |
                                  wpilib.XboxController |
                                  wpilib.PS4Controller |
                                  wpilib.PS5Controller]

