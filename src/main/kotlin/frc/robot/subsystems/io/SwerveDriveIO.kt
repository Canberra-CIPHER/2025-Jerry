package frc.robot.subsystems.io

import swervelib.SwerveDrive

data class SwerveDriveIO(
    val swerveDrive: SwerveDrive,
    val getForceSlow: (() -> Boolean)? = null,
)

