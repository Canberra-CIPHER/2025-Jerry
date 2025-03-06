package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.RotationSystem

class Superstructure(val elevator: Elevator, val arm: RotationSystem, val wrist: RotationSystem) {
    fun goToOriginCommand(): Command {
        return wrist.goToAngleCommand(0.0, false)
    }

    fun goToStowedCommand(): Command {
        return goToOriginCommand().andThen(
            wrist.goToAngleCommand(0.0, false).andThen(
                arm.goToAngleCommand(0.0, false).andThen(
                    elevator.goToHeightCommand(0.0, false))
                )
            )
    }

    fun loadFromStationCommand(): Command {
        return goToOriginCommand().andThen(
            elevator.goToHeightCommand(0.5, false)
                .alongWith(arm.goToAngleCommand(45.0, false))
        )
    }

    fun goToReefLevelCommand(level: Int): Command {
        var cmd = goToOriginCommand()
        when (level) {
            1 -> {
                cmd = cmd.andThen(
                    elevator.goToHeightCommand(0.0, false)
                        .alongWith(arm.goToAngleCommand(83.0, false))
                ).andThen(wrist.goToAngleCommand(90.0, false))
            }
            2 -> {
                cmd = cmd.andThen(
                    elevator.goToHeightCommand(0.0, false)
                        .alongWith(arm.goToAngleCommand(46.0, false))
                ).andThen(wrist.goToAngleCommand(90.0, false))
            }
            3 -> {
                cmd = cmd.andThen(
                    elevator.goToHeightCommand(0.522, false)
                        .alongWith(arm.goToAngleCommand(42.0, false))
                ).andThen(wrist.goToAngleCommand(90.0, false))
            }
            4 -> {
                cmd = cmd.andThen(
                    elevator.goToHeightCommand(1.10, false)
                        .alongWith(arm.goToAngleCommand(42.0, false))
                ).andThen(wrist.goToAngleCommand(90.0, false))
            }
            else -> {}
        }
        return cmd
    }
}