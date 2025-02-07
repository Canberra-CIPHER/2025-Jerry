package frc.robot.subsystems

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.io.GrabberIO

class Grabber (val io: GrabberIO, val holdVoltage: Double): SubsystemBase() {
    sealed class GrabberState {
        class EStop() : GrabberState()
        class Intake(val voltage: Double) : GrabberState()
        class Output(val voltage: Double) : GrabberState()
        class Hold() : GrabberState()
    }

    var state: GrabberState = GrabberState.Hold()

    fun estop() {
        this.state = GrabberState.EStop()
    }

    fun intake(voltage: Double) {
        this.state = GrabberState.Intake(voltage)
    }

    fun output(voltage: Double) {
        this.state = GrabberState.Intake(voltage)
    }

    fun hold() {
        this.state = GrabberState.Hold()
    }

    fun controlPeriodic() {
        var voltage = 0.0
        val currentState = state

        when (currentState) {
            is GrabberState.EStop -> voltage = 0.0
            is GrabberState.Intake -> {
                if (io.hasGamePiece != null) {
                    io.hasGamePiece.let { hasGamePiece ->
                        val hasPiece = hasGamePiece()
                        if (hasPiece == true) {
                            this.state = GrabberState.Hold()
                        }
                    }
                }
                voltage = currentState.voltage
            }
            is GrabberState.Output -> {
                voltage = -currentState.voltage
            }
            is GrabberState.Hold -> {
                voltage = this.holdVoltage
            }
        }

        io.voltageController.setVoltage(voltage)
    }

    override fun periodic() {
    }

    fun intakeCommand(voltage: Double): Command {
        return FunctionalCommand(
            { -> Unit },
            { -> this.intake(voltage) },
            { _ -> this.hold() },
            { -> this.state is GrabberState.Hold },
            this
        )
    }

    fun outputCommand(voltage: Double): Command {
        return FunctionalCommand(
            { -> Unit },
            { -> this.intake(-voltage) },
            { _ -> this.hold() },
            { -> false },
            this
        )
    }
}