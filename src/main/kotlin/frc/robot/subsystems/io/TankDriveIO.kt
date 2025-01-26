package frc.robot.subsystems.io

import edu.wpi.first.wpilibj.drive.DifferentialDrive

data class TankDriveIO(
    val diffDrive: DifferentialDrive,
    val getYaw: () -> Double,
)
