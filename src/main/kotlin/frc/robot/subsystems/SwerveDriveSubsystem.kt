package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.io.SwerveDriveIO
import swervelib.math.SwerveMath
import java.util.function.DoubleSupplier

class SwerveDriveSubsystem(val io: SwerveDriveIO) : SubsystemBase() {
    fun driveDefaultCommand(
        translationX: DoubleSupplier,
        translationY: DoubleSupplier,
        headingX: DoubleSupplier,
        headingY: DoubleSupplier,
    ): Command {
        return FunctionalCommand(
            { -> Unit },
            { ->
                val scaledInputs = SwerveMath.scaleTranslation(
                    Translation2d(
                        translationX.asDouble,
                        translationY.asDouble
                    ), 1.0
                )

                io.swerveDrive.driveFieldOriented(
                    io.swerveDrive.swerveController.getTargetSpeeds(
                        scaledInputs.x, scaledInputs.y,
                        headingX.asDouble,
                        headingY.asDouble,
                        io.swerveDrive.odometryHeading.radians,
                        io.swerveDrive.maximumModuleDriveVelocity
                    )
                )
            },
            { _: Boolean -> io.swerveDrive.drive(ChassisSpeeds(0.0, 0.0, 0.0)) },
            { -> false },
            this
        )
    }
}
