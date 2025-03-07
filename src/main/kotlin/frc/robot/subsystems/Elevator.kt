package frc.robot.subsystems

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.smartdashboard.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.io.ElevatorIO
import frc.robot.wrappers.WrappedSparkMax


class Elevator(
    val io: ElevatorIO,
    var pid: ProfiledPIDController,
    var feedforward: ElevatorFeedforward,
    var motor: DCMotor,
    var minHeight: Double,
    var maxHeight: Double,
    var elevatorModel: MechanismLigament2d,
) :  SubsystemBase() {
    sealed class ElevatorState {
        class EStop() : ElevatorState()
        class Hold(val height: Double) : ElevatorState()
        class Moving(val height: Double) : ElevatorState()
        class Init() : ElevatorState()
    }

    var state: ElevatorState = ElevatorState.Init()

    val elevatorErrorPublisher = NetworkTableInstance.getDefault().getTopic("elevator/error").genericPublish("double")
    val elevatorPositionPublisher = NetworkTableInstance.getDefault().getTopic("elevator/position").genericPublish("double")
    val elevatorVoltagePublisher = NetworkTableInstance.getDefault().getTopic("elevator/voltage").genericPublish("double")
    val elevatorCurrentPublisher = NetworkTableInstance.getDefault().getTopic("elevator/current").genericPublish("double")
    val elevatorCANPublisher = NetworkTableInstance.getDefault().getTopic("elevator/CAN").genericPublish("double")

    fun estop() {
        this.state = ElevatorState.EStop()
    }

    fun goToHeight(height: Double, reset: Boolean = true) {
      /*  if (reset) {
            this.pid.reset()
        }*/
        this.state = ElevatorState.Moving(height)
    }

    fun isStable(): Boolean {
        return this.pid.atSetpoint()
    }

    fun getCurrentHeight(): Double {
        return io.positionProvider.getPosition()
    }

    fun controlPeriodic() {
        var currentHeight = io.positionProvider.getPosition()
        var voltage = 0.0

        val state = this.state

        when(state) {
            is ElevatorState.EStop -> voltage = (0.0)
            is ElevatorState.Hold -> {
                var output = pid.calculate(currentHeight, state.height)
                voltage = output
                //voltage = (output + this.feedforward.calculate(state.height))

                if (!pid.atSetpoint()) {
                    this.state = ElevatorState.Moving(state.height)
                }
                else if (io.getCalibrationHeight != null) {
                    io.getCalibrationHeight.let { getCalibrationHeight ->
                        val calHeight = getCalibrationHeight()
                        if (calHeight != null) {
                            io.positionProvider.setPosition(calHeight)
                        }
                    }
                }
            }
            is ElevatorState.Moving -> {
                var output = pid.calculate(currentHeight, state.height)
                voltage = output
                //voltage = (output + this.feedforward.calculate(state.height))

                if (pid.atSetpoint()) {
                    this.state = ElevatorState.Hold(state.height)
                }
            }
            is ElevatorState.Init -> {
                if (io.getCalibrationHeight != null) {
                    io.getCalibrationHeight.let { getCalibrationHeight ->
                        val calHeight = getCalibrationHeight()
                        if (calHeight != null) {
                            io.positionProvider.setPosition(calHeight)
                        }
                    }
                }
                else {
                    io.positionProvider.setPosition(0.0)
                }
            }
        }

        if (io.getLimitLow?.invoke() == true && voltage < 0.0) {
            voltage = 0.0
            io.positionProvider.setPosition(0.0)
        }

        if (io.getLimitHigh?.invoke() == true && voltage > 0.0) {
            voltage = 0.0
        }

        io.voltageController.setVoltage(voltage)
    }

    override fun periodic() {
        elevatorErrorPublisher.setDouble(pid.positionError)
        elevatorPositionPublisher.setDouble(io.positionProvider.getPosition())
        elevatorVoltagePublisher.setDouble(io.voltageController.getVoltage())
        elevatorCurrentPublisher.setDouble(io.voltageController.getCurrent())

        io.getCalibrationHeight.let { getCalibrationHeight ->
            val calHeight = getCalibrationHeight?.let { it() }
            if (calHeight != null) {
                elevatorCANPublisher.setDouble(calHeight)
            }
        }
    }

    fun goToHeightCommand(height: Double, continuous: Boolean): Command {
        return FunctionalCommand(
            { -> this.goToHeight(height, true) },
            { -> Unit },
            { _ -> Unit },
            { -> !continuous && this.isStable() },
            this
        )
    }

    val sim = ElevatorSim(feedforward.kv, feedforward.ka, motor, minHeight, maxHeight, true, 0.0)

    override fun simulationPeriodic() {
        if (io.voltageController is WrappedSparkMax) {
            var simMotor = io.voltageController.sim
            simMotor?.iterate(sim.velocityMetersPerSecond, 12.0, 0.02)
        }

        println("Sim Periodic " + io.voltageController.getVoltage())

        sim.setInputVoltage(io.voltageController.getVoltage())
        sim.update(0.02)

        elevatorModel.length = sim.positionMeters
    }
}