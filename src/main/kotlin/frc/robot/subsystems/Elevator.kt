package frc.robot.subsystems

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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

    var mech: Mechanism2d = Mechanism2d(3.0, 3.0)
    var root: MechanismRoot2d = mech.getRoot("elevator", 2.0, 0.0)
    var jerryElevator = root.append(MechanismLigament2d("elevator", 0.0, 90.0))

    init {
        SmartDashboard.putData("ElevatorMech2d", mech)
    }

    fun estop() {
        this.state = ElevatorState.EStop()
    }

    fun goToHeight(height: Double, reset: Boolean = true) {
        /*if (reset) {
            this.pid.reset()
        }*/
        this.state = ElevatorState.Moving(height)
    }

    fun isStable(): Boolean {
        return this.pid.atSetpoint()
    }

    fun controlPeriodic() {
        var currentHeight = io.positionProvider.getPosition()
        var voltage = 0.0

        val state = this.state

        when(state) {
            is ElevatorState.EStop -> voltage = (0.0)
            is ElevatorState.Hold -> {
                var output = pid.calculate(currentHeight, state.height)
                voltage = (output + this.feedforward.calculate(state.height))

                if (!pid.atSetpoint()) {
                    this.state = ElevatorState.Moving(state.height)
                }
                else if (io.getCalibrationHeight != null) {
                    io.getCalibrationHeight.let { getCalibrationHeight ->
                        io.positionProvider.setPosition(getCalibrationHeight())
                    }
                }
            }
            is ElevatorState.Moving -> {
                var output = pid.calculate(currentHeight, state.height)
                voltage = (output + this.feedforward.calculate(state.height))

                if (pid.atSetpoint()) {
                    this.state = ElevatorState.Hold(state.height)
                }
            }
            is ElevatorState.Init -> {
                if (io.getCalibrationHeight != null) {
                    io.getCalibrationHeight.let { getCalibrationHeight ->
                        io.positionProvider.setPosition(getCalibrationHeight())
                    }
                }
            }
        }

        if (io.getLimitLow?.invoke() == true && voltage < 0.0) {
            voltage = 0.0
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
    }

    fun goToHeightCommand(height: Double, continuous: Boolean): Command {
        return FunctionalCommand(
            { -> this.goToHeight(height, true) },
            { -> Unit },
            { _ -> Unit },
            { -> !continuous && this.isStable() }
        )
    }

    val sim = ElevatorSim(feedforward.kv, feedforward.ka, motor, minHeight, maxHeight, true, 0.0)

    override fun simulationPeriodic() {
        if (io.voltageController is WrappedSparkMax) {
            var simMotor = io.voltageController.sim
            simMotor?.iterate(Units.radiansPerSecondToRotationsPerMinute(sim.velocityMetersPerSecond / (2.82 / 2.0 * 0.0254)), 12.0, 0.02)
        }

        println("Sim Periodic " + io.voltageController.getVoltage())

        sim.setInputVoltage(io.voltageController.getVoltage())
        sim.update(0.02)

        jerryElevator.length = sim.positionMeters
    }
}