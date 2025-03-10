package frc.robot.subsystems

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.RotationSystem.ArmState
import frc.robot.subsystems.io.SwerveDriveIO
import org.photonvision.PhotonCamera
import swervelib.math.SwerveMath
import java.util.function.DoubleSupplier
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min

class SwerveDriveSubsystem(
    val io: SwerveDriveIO,
    var pidX: ProfiledPIDController,
    var pidY: ProfiledPIDController,
    var camera: PhotonCamera? = null,
    var robotOriginToCameraTransform: Transform3d? = null
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
            val pose: Pose2d
        ) : DriveState()
    }

    var state: DriveState = DriveState.Manual(0.0, 0.0, 0.0, 0.0)

    fun estop() {
        this.state = DriveState.EStop()
    }

    fun goToPosition(pose: Pose2d) {
        this.state = DriveState.Position(pose)
    }

    fun manualControl(tX: Double, tY: Double, hX: Double, hY: Double) {
        this.state = DriveState.Manual(tX, tY, hX, hY)
    }

    fun isStable(): Boolean {
        return this.pidX.atSetpoint() && this.pidY.atSetpoint()
    }

    fun currentPose(): Pose2d {
        return io.swerveDrive.pose
    }

    override fun periodic() {
        for (result in camera?.allUnreadResults ?: emptyList()) {
            if (result.hasTargets()) {
                val fullTransform = result.bestTarget.bestCameraToTarget.plus(robotOriginToCameraTransform!!)
                io.swerveDrive.addVisionMeasurement(Pose2d(fullTransform.translation.toTranslation2d(), fullTransform.rotation.toRotation2d()), result.timestampSeconds)
            }
        }
    }

    fun controlPeriodic() {
        val state = this.state
        var speedFactor = 0.75

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
                var translationX = pidX.calculate(io.swerveDrive.pose.x, state.pose.x)
                var translationY = pidY.calculate(io.swerveDrive.pose.y, state.pose.y)

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
                        state.pose.rotation.radians,
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
        headingY: DoubleSupplier
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
        pose: Pose2d
    ): Command {
        return FunctionalCommand(
            { -> goToPosition(pose) },
            { -> Unit },
            { _: Boolean -> io.swerveDrive.drive(ChassisSpeeds(0.0, 0.0, 0.0)) },
            { -> isStable() },
            this
        )
    }
}