package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import frc.robot.subsystems.io.RotationSystemIO

class RotationSystem(val io: RotationSystemIO, var pid: ProfiledPIDController) :  SubsystemBase() {
    sealed class ArmState {
        class EStop() : ArmState()
        class Hold(val angle: Double) : ArmState()
        class Moving(val angle: Double) : ArmState()
    }

    var state: ArmState = ArmState.Hold(0.0)

    val armErrorPublisher = NetworkTableInstance.getDefault().getTopic("arm/error").genericPublish("double")
    val armPositionPublisher = NetworkTableInstance.getDefault().getTopic("arm/position").genericPublish("double")
    val armVoltagePublisher = NetworkTableInstance.getDefault().getTopic("arm/voltage").genericPublish("double")

    fun estop() {
        this.state = ArmState.EStop()
    }

    fun goToAngle(angle: Double) {
        this.state = ArmState.Moving(angle)
    }

    fun isStable(): Boolean {
        return this.pid.atSetpoint()
    }

    fun controlPeriodic() {
        var currentAngle = io.positionProvider.getPosition()
        var voltage = 0.0

        val state = this.state

        when(state) {
            is ArmState.EStop -> voltage = 0.0
            is ArmState.Hold -> {
                var output = pid.calculate(currentAngle, state.angle)
                voltage = output

                if (!pid.atSetpoint()) {
                    this.state = ArmState.Moving(state.angle)
                }
            }
            is ArmState.Moving -> {
                var output = pid.calculate(currentAngle, state.angle)
                voltage = output

                if (pid.atSetpoint()) {
                    this.state = ArmState.Hold(state.angle)
                }
            }
        }

        this.io.voltageController.setVoltage(voltage)
    }

    override fun periodic() {
        armErrorPublisher.setDouble(pid.positionError)
        armPositionPublisher.setDouble(io.positionProvider.getPosition())
        armVoltagePublisher.setDouble(io.voltageController.getVoltage())
    }

    fun goToAngleCommand(angle: Double): Command {
        return FunctionalCommand(
            { -> this.goToAngle(angle) },
            { -> Unit },
            { _ -> Unit },
            { -> this.isStable() }
        )
    }
}