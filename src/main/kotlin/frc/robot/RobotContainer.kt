package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.XboxController
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.spark.SparkBase
import frc.robot.subsystems.TankDrive

class RobotContainer {
    init {
        configureBindings()
    }

    private fun configureBindings() {}

    //val autonomousCommand: Command
        //get() = Commands.print("No autonomous command configured")

    val xbox = XboxController(0)

    val leftDrive1 = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    val leftDrive2 = SparkMax(1, SparkLowLevel.MotorType.kBrushless)
    val rightDrive1 = SparkMax(2, SparkLowLevel.MotorType.kBrushless)
    val rightDrive2 = SparkMax(3, SparkLowLevel.MotorType.kBrushless)

    init {
        var configLeft = SparkMaxConfig()
        configLeft.follow(leftDrive1)
        leftDrive2.configure(configLeft, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        var configRight = SparkMaxConfig()
        configRight.follow(rightDrive1)
        rightDrive2.configure(configRight, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    val diffDrive = DifferentialDrive(leftDrive1, rightDrive1)

    val driveSystem = TankDrive(diffDrive)
}
