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
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.RotationSystem
import frc.robot.subsystems.TankDrive
import frc.robot.subsystems.io.ElevatorIO
import frc.robot.subsystems.io.RotationSystemIO
import frc.robot.subsystems.io.TankDriveIO
import frc.robot.wrappers.WrappedSparkMax

class RobotContainer {
    val loop = EventLoop()

    //val autonomousCommand: Command
        //get() = Commands.print("No autonomous command configured")

    val xbox = XboxController(0)

    /*val leftDrive1 = SparkMax(1, SparkLowLevel.MotorType.kBrushed)
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
    }*/

/*    val liftMotor1 = SparkMax(10, SparkLowLevel.MotorType.kBrushless)

    init {
        var configLift1 = SparkMaxConfig()
        configLift1.inverted(false)
        configLift1.encoder.positionConversionFactor(0.035814)
        configLift1.encoder.velocityConversionFactor(0.035814)
        configLift1.smartCurrentLimit(40)
        liftMotor1.configure(configLift1, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    val liftMotorModel = DCMotor.getNEO(1).withReduction(15.0)

    val liftMotor1Wrapped = WrappedSparkMax(liftMotor1, liftMotorModel)

    val liftIO = ElevatorIO(
        liftMotor1Wrapped,
        liftMotor1Wrapped,
    )

    val elevatorPID = ProfiledPIDController(12.0, 0.0, 0.0, TrapezoidProfile.Constraints(2.0, 16.0))
    val elevatorFeedforward = ElevatorFeedforward(0.0, 1.54, 8.16, 0.19)

    init {
        elevatorPID.setTolerance(0.02)
        elevatorPID.iZone = 0.1
        SmartDashboard.putData("Elevator PID", elevatorPID)
    }

    val elevator = Elevator(
        liftIO,
        elevatorPID,
        elevatorFeedforward,
        liftMotorModel,
        0.0,
        1.8)

    init {
        xbox.a(loop).rising().ifHigh {
            elevator.goToHeightCommand(1.0, false).schedule()
        }
        xbox.b(loop).rising().ifHigh {
            elevator.goToHeightCommand(0.0, false).schedule()
        }
    }*/

    val armMotor1 = SparkMax(20, SparkLowLevel.MotorType.kBrushless)

    init {
        var configArm1 = SparkMaxConfig()
        configArm1.inverted(false)
        configArm1.encoder.positionConversionFactor(360.0)
        configArm1.encoder.velocityConversionFactor(360.0)
        configArm1.smartCurrentLimit(40)
        armMotor1.configure(configArm1, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    val armMotorModel = DCMotor.getNEO(1).withReduction(1.0)
    val armMotor1Wrapped = WrappedSparkMax(armMotor1, armMotorModel)

    val armIO = RotationSystemIO(
        armMotor1Wrapped,
        armMotor1Wrapped,
    )

    val armPID = ProfiledPIDController(2.0 / 180.0, 0.0, 0.0, TrapezoidProfile.Constraints(60.0, 1400.0))

    init {
        armPID.setTolerance(0.5)
        armPID.iZone = 3.0
        SmartDashboard.putData("Arm PID", armPID)
    }

    val arm = RotationSystem(
        armIO,
        armPID,
        )

    init {
        xbox.x(loop).rising().ifHigh {
            arm.goToAngleCommand(180.0).schedule()
        }
        xbox.y(loop).rising().ifHigh {
            arm.goToAngleCommand(-180.0).schedule()
        }
    }
}
