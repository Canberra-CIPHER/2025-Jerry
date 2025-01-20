package frc.robot.subsystems

import com.revrobotics.spark.SparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ElevatorFeedforward

class Elevator(val liftMotor: SparkMax, var pid: PIDController, var feedforward: ElevatorFeedforward) :  SubsystemBase() {
    sealed class ElevatorState {
        class EStop() : ElevatorState()
        class Hold(val height: Double) : ElevatorState()
        class Moving(val height: Double) : ElevatorState()
    }

    var state: ElevatorState = ElevatorState.Hold(0.0)

    fun estop() {
        this.state = ElevatorState.EStop()
    }

    fun goToHeight(height: Double) {
        this.state = ElevatorState.Moving(height)
    }

    fun isStable(): Boolean {
        return this.pid.atSetpoint()
    }

    override fun periodic() {
        var currentHeight = liftMotor.encoder.position

        val state = this.state

        when(state) {
            is ElevatorState.EStop -> this.liftMotor.set(0.0)
            is ElevatorState.Hold -> {
                var output = pid.calculate(currentHeight, state.height)
                this.liftMotor.setVoltage(output + this.feedforward.calculate(state.height))

                if (!pid.atSetpoint()) {
                    this.state = ElevatorState.Moving(state.height)
                }
            }
            is ElevatorState.Moving -> {
                var output = pid.calculate(currentHeight, state.height)
                this.liftMotor.setVoltage(output + this.feedforward.calculate(state.height))

                if (pid.atSetpoint()) {
                    this.state = ElevatorState.Hold(state.height)
                }
            }
        }
    }
}