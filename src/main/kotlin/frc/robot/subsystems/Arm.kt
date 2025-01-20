package frc.robot.subsystems

import com.revrobotics.spark.SparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.controller.PIDController

class Arm(val armMotor: SparkMax, var pid: PIDController) :  SubsystemBase() {
    sealed class ArmState {
        class EStop() : ArmState()
        class Hold(val angle: Double) : ArmState()
        class Moving(val angle: Double) : ArmState()
    }

    var state: ArmState = ArmState.Hold(0.0)

    fun estop() {
        this.state = ArmState.EStop()
    }

    fun goToAngle(angle: Double) {
        this.state = ArmState.Moving(angle)
    }

    fun isStable(): Boolean {
        return this.pid.atSetpoint()
    }

    override fun periodic() {
        var currentAngle = armMotor.encoder.position

        val state = this.state

        when(state) {
            is ArmState.EStop -> this.armMotor.set(0.0)
            is ArmState.Hold -> {
                var output = pid.calculate(currentAngle, state.angle)
                this.armMotor.setVoltage(output)

                if (!pid.atSetpoint()) {
                    this.state = ArmState.Moving(state.angle)
                }
            }
            is ArmState.Moving -> {
                var output = pid.calculate(currentAngle, state.angle)
                this.armMotor.setVoltage(output)

                if (pid.atSetpoint()) {
                    this.state = ArmState.Hold(state.angle)
                }
            }
        }
    }
}