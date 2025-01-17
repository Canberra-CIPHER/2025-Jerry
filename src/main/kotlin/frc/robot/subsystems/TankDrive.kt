package frc.robot.subsystems

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.Command

class TankDrive(val diffDrive: DifferentialDrive) : SubsystemBase() {
    fun setThrottle(left: Double, right: Double) {
        diffDrive.tankDrive(left, right, false)
    }

    fun driveDefaultCommand(xboxController: XboxController): Command {
        return FunctionalCommand(
            { -> Unit },
            { -> setThrottle(xboxController.getLeftY(), xboxController.getRightY()) },
            { _: Boolean -> setThrottle(0.0, 0.0) },
            { -> false },
            this
        )
    }
}