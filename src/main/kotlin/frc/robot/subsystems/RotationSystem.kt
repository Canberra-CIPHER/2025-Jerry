package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import frc.robot.subsystems.io.RotationSystemIO
import frc.robot.wrappers.WrappedSparkMax

class RotationSystem(
    val io: RotationSystemIO,
    var pid: ProfiledPIDController,
    var motor: DCMotor,
    var armModel: MechanismLigament2d,
    val systemName: String
) :  SubsystemBase() {
    sealed class ArmState {
        class EStop() : ArmState()
        class Init() : ArmState()
        class Hold(val angle: Double) : ArmState()
        class Moving(val angle: Double) : ArmState()
    }

    var state: ArmState = ArmState.Init()

    val armErrorPublisher = NetworkTableInstance.getDefault().getTopic(systemName + "/error").genericPublish("double")
    val armPositionPublisher = NetworkTableInstance.getDefault().getTopic(systemName + "/position").genericPublish("double")
    val armVoltagePublisher = NetworkTableInstance.getDefault().getTopic(systemName + "/voltage").genericPublish("double")

    fun estop() {
        this.state = ArmState.EStop()
    }

    fun goToAngle(angle: Double) {
        this.state = ArmState.Moving(angle)
    }

    fun isStable(): Boolean {
        return this.pid.atSetpoint()
    }

    fun getCurrentAngle(): Double {
        return io.positionProvider.getPosition()
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
            is ArmState.Init -> {
                io.positionProvider.setPosition(0.0)
                this.state = ArmState.Hold(0.0)
            }
        }

        this.io.voltageController.setVoltage(voltage)
    }

    override fun periodic() {
        armErrorPublisher.setDouble(pid.positionError)
        armPositionPublisher.setDouble(io.positionProvider.getPosition())
        armVoltagePublisher.setDouble(io.voltageController.getVoltage())
    }

    fun goToAngleCommand(angle: Double, continuous: Boolean): Command {
        return FunctionalCommand(
            { -> this.goToAngle(angle) },
            { -> Unit },
            { _ -> Unit },
            { -> !continuous && this.isStable() },
            this
        )
    }

    val sim = SingleJointedArmSim(motor, 1.0, 0.6, 0.4, 0.0, 6.28, true, 0.0, 0.0, 0.0)

    override fun simulationPeriodic() {
        if (io.voltageController is WrappedSparkMax) {
            var simMotor = io.voltageController.sim
            simMotor?.iterate(sim.velocityRadPerSec / 6.28 * 360.0, 12.0, 0.02)
        }

        println("Sim Periodic " + sim.velocityRadPerSec)

        sim.setInputVoltage(io.voltageController.getVoltage())
        sim.update(0.02)

        armModel.angle = sim.angleRads / 6.28 * 360.0 - 90.0
    }
}