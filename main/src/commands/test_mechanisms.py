import commands2
import commands
import subsystems


class TestMechanisms(commands2.Command):
    climber: subsystems.Climber
    _command: commands2.Command

    def __init__(self, climber: subsystems.Climber):
        super().__init__()
        test_time = 1.0
        self._intake = intake
        self._shooter = shooter
        self._indexer = indexer
        self._climber = climber
        self._command = commands2.SequentialCommandGroup(
            commands.ClimberUp(self._climber),
            commands2.WaitCommand(test_time / 2.0),
            commands.ClimberStop(self._climber),
            commands2.WaitCommand(test_time / 2.0),
            commands.ClimberDown(self._climber),
            commands2.WaitCommand(test_time / 2.0),
            commands.ClimberStop(self._climber)

        )

    def execute(self):
        commands2.CommandScheduler.getInstance().schedule(self._command)
