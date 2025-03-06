package frc.robot.subsystems

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.RotationSystem.ArmState
import frc.robot.subsystems.io.SwerveDriveIO
import swervelib.math.SwerveMath
import java.util.function.DoubleSupplier
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min

class SwerveDriveSubsystem(
    val io: SwerveDriveIO,
    var pidX: ProfiledPIDController,
    var pidY: ProfiledPIDController,
) : SubsystemBase() {
    sealed class DriveState {
        class EStop() : DriveState()
        class Manual(
            val translationX: Double,
            val translationY: Double,
            val headingX: Double,
            val headingY: Double,
        ) : DriveState()
        class Position(
            val positionX: Double,
            val positionY: Double,
            val headingX: Double,
            val headingY: Double,
        ) : DriveState()
    }

    var state: DriveState = DriveState.Manual(0.0, 0.0, 0.0, 0.0)

    fun estop() {
        this.state = DriveState.EStop()
    }

    fun goToPosition(tX: Double, tY: Double, hX: Double, hY: Double) {
        this.state = DriveState.Position(tX, tY, hX, hY)
    }

    fun manualControl(tX: Double, tY: Double, hX: Double, hY: Double) {
        this.state = DriveState.Manual(tX, tY, hX, hY)
    }

    fun isStable(pid: ProfiledPIDController): Boolean {
        return pid.atSetpoint()
    }

    fun controlPeriodic() {
        val state = this.state
        var speedFactor = 1.0

        if (io.getForceSlow?.invoke() == true) {
            speedFactor = 0.5
        }

        when (state) {
            is DriveState.EStop -> { io.swerveDrive.drive(ChassisSpeeds(0.0, 0.0, 0.0)) }
            is DriveState.Manual -> {
                val scaledInputs = SwerveMath.scaleTranslation(
                    Translation2d(
                        state.translationX,
                        state.translationY
                    ), speedFactor
                )

                io.swerveDrive.driveFieldOriented(
                    io.swerveDrive.swerveController.getTargetSpeeds(
                        scaledInputs.x,
                        scaledInputs.y,
                        state.headingX,
                        state.headingY,
                        io.swerveDrive.odometryHeading.radians,
                        io.swerveDrive.maximumModuleDriveVelocity
                    )
                )
            }

            is DriveState.Position -> {
                var translationX = pidX.calculate(io.swerveDrive.pose.x, state.positionX)
                var translationY = pidY.calculate(io.swerveDrive.pose.y, state.positionY)

                val clampToFactor = { x: Double -> min(
                    max(x, -speedFactor * io.swerveDrive.maximumChassisVelocity),
                    speedFactor * io.swerveDrive.maximumChassisVelocity
                ) }

                translationX = clampToFactor(translationX)
                translationY = clampToFactor(translationY)

                /*val scaledInputs = SwerveMath.(
                    Translation2d(
                        translationX,
                        translationY
                    ), 0.8
                )*/

                io.swerveDrive.driveFieldOriented(
                    io.swerveDrive.swerveController.getRawTargetSpeeds(
                        translationX,
                        translationY,
                        Math.atan2(state.headingY, state.headingX),
                        io.swerveDrive.odometryHeading.radians
                    )
                )
            }
        }
    }

    fun driveDefaultCommand(
        translationX: DoubleSupplier,
        translationY: DoubleSupplier,
        headingX: DoubleSupplier,
        headingY: DoubleSupplier,
        isRelative: Boolean,
    ): Command {
        return FunctionalCommand(
            { -> Unit },
            { -> manualControl(translationX.asDouble, translationY.asDouble, headingX.asDouble, headingY.asDouble) },
            { _: Boolean -> io.swerveDrive.drive(ChassisSpeeds(0.0, 0.0, 0.0)) },
            { -> false },
            this
        )
    }

    fun driveToPosition(
        translationX: Double,
        translationY: Double,
        headingX: Double,
        headingY: Double,
    ): Command {
        return FunctionalCommand(
            { -> Unit },
            { -> goToPosition(translationX, translationY, headingX, headingY) },
            { _: Boolean -> io.swerveDrive.drive(ChassisSpeeds(0.0, 0.0, 0.0)) },
            { -> false },
            this
        )
    }
}