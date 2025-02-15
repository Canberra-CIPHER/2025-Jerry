package frc.robot.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.io.TankDriveIO

class TankDrive(val io: TankDriveIO, var anglePID: PIDController) : SubsystemBase() {
    val publisherTurnPower = NetworkTableInstance.getDefault().getTopic("turnPower").genericPublish("double")

    fun setThrottle(left: Double, right: Double, squareInputs: Boolean) {
        io.diffDrive.tankDrive(left, right, squareInputs)
    }

    fun driveDefaultCommand(xboxController: XboxController): Command {
        return FunctionalCommand(
            { -> Unit },
            { -> setThrottle(xboxController.getLeftY(), xboxController.getRightY(), true) },
            { _: Boolean -> setThrottle(0.0, 0.0, false) },
            { -> false },
            this
        )
    }

    /*fun autoDefaultCommand(): Command {
        var autoTimer = Timer()
        return FunctionalCommand(
            { -> autoTimer.start() },
            { -> setThrottle(0.5, 0.5)},
            { _: Boolean -> setThrottle(0.0, 0.0) },
            { -> autoTimer.get() > 3},
            this
        )
    }*/

    fun snapToAngleCommand(angle: Double, continuous: Boolean): Command {
        return FunctionalCommand(
            { ->
                anglePID.reset()
                anglePID.setpoint = angle
                anglePID.enableContinuousInput(0.0, 360.0)
            },
            { ->
                var turnPower = anglePID.calculate(-io.getYaw())
                if (turnPower > 0.25) {
                    turnPower = 0.25
                }
                else if (turnPower < -0.25) {
                    turnPower = -0.25
                }
                publisherTurnPower.setDouble(turnPower)
                setThrottle(turnPower, -turnPower, false)
            },
            { _: Boolean -> setThrottle(0.0, 0.0, false) },
            { -> !continuous && anglePID.atSetpoint()},
            this
        )
    }
}