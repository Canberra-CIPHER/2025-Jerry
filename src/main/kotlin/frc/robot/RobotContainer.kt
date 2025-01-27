package frc.robot

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.XboxController
import com.studica.frc.AHRS;
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.spark.SparkBase
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.TankDrive
import frc.robot.subsystems.io.ElevatorIO
import frc.robot.subsystems.io.TankDriveIO

class RobotContainer {
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

        SmartDashboard.putData("Gyro", gyro)
    }

    val diffDrive = DifferentialDrive(leftDrive1, rightDrive1)
    val driveAnglePID = PIDController(0.07, 0.01, 0.002)
    val driveIO = TankDriveIO(diffDrive, { -> gyro.yaw.toDouble() })
    val driveSystem = TankDrive(driveIO, driveAnglePID)

    init {
        driveAnglePID.setTolerance(1.0)
        driveAnglePID.iZone = 10.0
        SmartDashboard.putData("DriveAngle PID", driveAnglePID);

        for (angle in (0..360 step 45)) {
            xbox.pov(angle, loop).rising().ifHigh {
                driveSystem.snapToAngleCommand(angle.toDouble(), false).schedule()
            }
        }
    }

    val liftMotor1 = SparkMax(10, SparkLowLevel.MotorType.kBrushless)

    init {
        var configLift1 = SparkMaxConfig()
        configLift1.inverted(false)
        configLift1.encoder.positionConversionFactor(0.035814)
        configLift1.encoder.velocityConversionFactor(0.035814)
        configLift1.smartCurrentLimit(40)
        leftDrive1.configure(configLift1, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    val liftIO = ElevatorIO(
        { -> liftMotor1.busVoltage * liftMotor1.appliedOutput },
        { voltage: Double -> liftMotor1.setVoltage(voltage) },
        { -> liftMotor1.encoder.position },
        { height: Double -> liftMotor1.encoder.position = height },
    )

    val elevatorPID = PIDController(12.0, 0.0, 0.0)
    val elevatorFeedforward = ElevatorFeedforward(0.0, 1.54, 8.16, 0.19)

    init {
        elevatorPID.setTolerance(0.02)
        elevatorPID.iZone = 0.1
        SmartDashboard.putData("Elevator PID", elevatorPID);
    }

    val elevator = Elevator(
        liftIO,
        elevatorPID,
        elevatorFeedforward,
        DCMotor.getNEO(1).withReduction(15.0),
        0.0,
        1.8)

    init {
        xbox.a(loop).rising().ifHigh {
            elevator.goToHeightCommand(1.0, false).schedule()
        }
        xbox.b(loop).rising().ifHigh {
            elevator.goToHeightCommand(0.0, false).schedule()
        }
    }
}
