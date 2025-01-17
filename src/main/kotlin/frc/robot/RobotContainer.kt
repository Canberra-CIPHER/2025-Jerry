package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel

class RobotContainer {
    init {
        configureBindings()
    }

    private fun configureBindings() {}

    val autonomousCommand: Command
        get() = Commands.print("No autonomous command configured")

    val leftDrive1 = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    val leftDrive2 = SparkMax(1, SparkLowLevel.MotorType.kBrushless)
    val rightDrive1 = SparkMax(2, SparkLowLevel.MotorType.kBrushless)
    val rightDrive2 = SparkMax(3, SparkLowLevel.MotorType.kBrushless)
}
