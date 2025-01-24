package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.XboxController
import com.studica.frc.AHRS;
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.spark.SparkBase
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NTSendable
import edu.wpi.first.networktables.NTSendableBuilder
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.event.EventLoop
import frc.robot.subsystems.TankDrive

class RobotContainer {
    init {
        configureBindings()
    }

    private fun configureBindings() {}

    val loop = EventLoop()

    //val autonomousCommand: Command
        //get() = Commands.print("No autonomous command configured")

    val xbox = XboxController(0)

    val leftDrive1 = SparkMax(1, SparkLowLevel.MotorType.kBrushed)
    val leftDrive2 = SparkMax(2, SparkLowLevel.MotorType.kBrushed)
    val rightDrive1 = SparkMax(3, SparkLowLevel.MotorType.kBrushed)
    val rightDrive2 = SparkMax(4, SparkLowLevel.MotorType.kBrushed)

    val gyro = AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k200Hz)

    init {
        var configLeft1 = SparkMaxConfig()
        configLeft1.inverted(false)
        leftDrive1.configure(configLeft1, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        var configLeft2 = SparkMaxConfig()
        configLeft2.follow(leftDrive1)
        leftDrive2.configure(configLeft2, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        var configRight1 = SparkMaxConfig()
        configRight1.inverted(true)
        rightDrive1.configure(configRight1, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        var configRight2 = SparkMaxConfig()
        configRight2.follow(rightDrive1)
        rightDrive2.configure(configRight2, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    val diffDrive = DifferentialDrive(leftDrive1, rightDrive1)
    val driveAnglePID = PIDController(5.0/180.0, 0.0, 0.0)
    val driveSystem = TankDrive(diffDrive, gyro, driveAnglePID)

    init {
        driveAnglePID.setTolerance(5.0)
        xbox.povLeft(loop).rising().ifHigh {
            driveSystem.snapToAngleCommand(45.0, false).schedule()
        }
    }
}
