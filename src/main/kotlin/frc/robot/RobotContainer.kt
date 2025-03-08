package frc.robot

import au.grapplerobotics.LaserCan
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.XboxController
import com.studica.frc.AHRS;
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.spark.SparkBase
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.cscore.UsbCamera
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.subsystems.*
import frc.robot.subsystems.io.*
import frc.robot.wrappers.WrappedSparkMax
import frc.robot.wrappers.WrappedTalonFX
import org.photonvision.PhotonCamera
import swervelib.SwerveDrive
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File
import kotlin.math.absoluteValue

class RobotContainer {
    val loop = EventLoop()

    //val autonomousCommand: Command
    //get() = Commands.print("No autonomous command configured")

    val vision = PhotonCamera("camera")
    val leds = AddressableLED(0)
    val xbox = XboxController(0)
    val buttonBoard = GenericHID(1)

    val swerveJsonDirectory = File(Filesystem.getDeployDirectory(),"swerve")
    val swerveDrive = SwerveParser(swerveJsonDirectory).createSwerveDrive(Units.feetToMeters(17.0))

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH
        CameraServer.startAutomaticCapture()

        var colour = AddressableLEDBuffer(120)
        var redValue = 10
        var greenValue = 0
        var blueValue = 10

        for (index in 0..119) {
            colour.setRGB(index, redValue, greenValue, blueValue)

            redValue += 10

            if (redValue > 255) {
                redValue = 0
            }
        }
        leds.setLength(120)
        leds.setData(colour)
        leds.start()
    }

    val driveXPID = ProfiledPIDController(5.0, 0.0, 0.0, TrapezoidProfile.Constraints(1.0, 30.0))
    val driveYPID = ProfiledPIDController(5.0, 0.0, 0.0, TrapezoidProfile.Constraints(1.0, 30.0))

    val swerveDriveIO = SwerveDriveIO(swerveDrive, { -> elevator.getCurrentHeight() > 0.2})
    val robotOriginToCameraTransform = Transform3d(
        Translation3d(0.0, 0.0, 0.0),
        Rotation3d(0.0, 0.0, 0.0)
    )
    val swerveDriveSystem = SwerveDriveSubsystem(swerveDriveIO, driveXPID, driveYPID /*, vision, robotOriginToCameraTransform */)

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

    var mech: Mechanism2d = Mechanism2d(3.0, 3.0)
    var root: MechanismRoot2d = mech.getRoot("elevator", 2.0, 0.0)
    var jerryElevator = root.append(MechanismLigament2d("elevator", 0.0, 90.0, 10.0, Color8Bit(Color.kDarkGoldenrod)))
    var jerryArm = jerryElevator.append(MechanismLigament2d("arm", 0.4, 0.0, 8.0, Color8Bit(Color.kGoldenrod)))
    var jerryTwist = jerryArm.append(MechanismLigament2d("twist", 0.1, 0.0, 8.0, Color8Bit(Color.kYellow)))
    var jerryWrist = jerryTwist.append(MechanismLigament2d("wrist", 0.4, 0.0, 8.0, Color8Bit(Color.kLemonChiffon)))

    init {
        SmartDashboard.putData("Mech2d", mech)
    }

    val liftMotor1 = TalonFX(20)
    val liftMotor2 = TalonFX(21)

    var liftConversion = 6.28 / 15.0 * (1.105 * 0.0254) * 2.0

    init {
        val outputConfigs = MotorOutputConfigs()
        outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive
        liftMotor1.configurator.apply(outputConfigs)
        liftMotor2.configurator.apply(outputConfigs)

        liftMotor2.setControl(Follower(liftMotor1.deviceID, false))
        val limitConfig = CurrentLimitsConfigs()
        limitConfig.StatorCurrentLimit = 60.0
        limitConfig.SupplyCurrentLimit = 30.0
        limitConfig.StatorCurrentLimitEnable = true
        limitConfig.SupplyCurrentLimitEnable = true
        liftMotor1.configurator.apply(limitConfig)
        liftMotor2.configurator.apply(limitConfig)

        val encoderConfig = FeedbackConfigs()
        encoderConfig.SensorToMechanismRatio = 1.0
        encoderConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor
        liftMotor1.configurator.apply(encoderConfig)
        liftMotor2.configurator.apply(encoderConfig)
    }

    val wrappedLiftMotor = WrappedTalonFX(liftMotor1, 1.0 / liftConversion)

    //val liftMotor1 = SparkMax(10, SparkLowLevel.MotorType.kBrushless)

    /*init {
        var conversion = 6.28 / 15.0 * (2.82/2.0 * 0.0254)
        var configLift1 = SparkMaxConfig()
        configLift1.inverted(false)
        configLift1.encoder.positionConversionFactor(conversion)
        configLift1.encoder.velocityConversionFactor(conversion / 60.0)
        configLift1.smartCurrentLimit(40)
        liftMotor1.configure(
            configLift1,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
    }*/

    val liftMotorModel = DCMotor.getNEO(1).withReduction(15.0)
    val liftLaserCAN = LaserCan(0)
    val liftFilter = LinearFilter.movingAverage(10)

    //val liftMotor1Wrapped = WrappedSparkMax(liftMotor1, liftMotorModel)

    val liftIO = ElevatorIO(
        wrappedLiftMotor,
        wrappedLiftMotor,
        getLimitLow = { ->
            val measurement = liftLaserCAN.measurement?.distance_mm
            if (measurement != null) {
                liftFilter.calculate(measurement.toDouble()) < 30.0
            }
            else {
                null
            }
        }
        //getCalibrationHeight = { -> liftLaserCAN.measurement?.distance_mm?.div(1000.0) }
    )

    val elevatorPID = ProfiledPIDController(20.0, 5.0, 0.0, TrapezoidProfile.Constraints(2.0, 16.0))
    // TODO: Check this
    val elevatorFeedforward = ElevatorFeedforward(0.0, 0.03, 10.21, 0.01)

    init {
        elevatorPID.setTolerance(0.05, 0.1)
        elevatorPID.iZone = 0.5
        SmartDashboard.putData("Elevator PID", elevatorPID)
    }

    val elevator = Elevator(
        liftIO,
        elevatorPID,
        elevatorFeedforward,
        liftMotorModel,
        0.0,
        1.8,
        jerryElevator
    )

    val armMotor1 = SparkMax(50, SparkLowLevel.MotorType.kBrushless)

    init {
        var configArm1 = SparkMaxConfig()
        configArm1.inverted(false)
        configArm1.encoder.positionConversionFactor(360.0 / 60.0)
        configArm1.encoder.velocityConversionFactor(360.0 / 60.0 / 60.0)
        configArm1.smartCurrentLimit(60)
        armMotor1.configure(
            configArm1,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
    }

    val armMotorModel = DCMotor.getNEO(1).withReduction(60.0)
    val armMotor1Wrapped = WrappedSparkMax(armMotor1, armMotorModel)

    val armIO = RotationSystemIO(
        armMotor1Wrapped,
        armMotor1Wrapped
    )

    val armPID = ProfiledPIDController(0.1, 0.05, 0.0, TrapezoidProfile.Constraints(100.0, 800.0))

    init {
        armPID.setTolerance(2.0, 0.5)
        armPID.iZone = 3.0
        SmartDashboard.putData("Arm PID", armPID)
    }

    val arm = RotationSystem(
        armIO,
        armPID,
        armMotorModel,
        jerryArm,
        "arm"
    )

    val twistMotor1 = SparkMax(2, SparkLowLevel.MotorType.kBrushless)

    init {
        var configTwist1 = SparkMaxConfig()
        configTwist1.inverted(false)
        configTwist1.encoder.positionConversionFactor(360.0 / 25.0)
        configTwist1.encoder.velocityConversionFactor(360.0 / 25.0 / 60.0)
        configTwist1.smartCurrentLimit(80)
        configTwist1.disableFollowerMode()
        twistMotor1.configure(
            configTwist1,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
    }

    val twistMotorModel = DCMotor.getNEO(1).withReduction(25.0)
    val twistMotor1Wrapped = WrappedSparkMax(twistMotor1, twistMotorModel)

    val twistIO = RotationSystemIO(
        twistMotor1Wrapped,
        twistMotor1Wrapped
    )

    val twistPID = ProfiledPIDController(0.1, 0.05, 0.0, TrapezoidProfile.Constraints(360.0, 1400.0))

    init {
        twistPID.setTolerance(5.0, 0.5)
        twistPID.iZone = 3.0
        SmartDashboard.putData("Twist PID", twistPID)
    }

    val twist = RotationSystem(
        twistIO,
        twistPID,
        twistMotorModel,
        jerryTwist,
        "twist"
    )

    val coralGrabberMotor1 = SparkMax(20, SparkLowLevel.MotorType.kBrushless)

    init {
        var configCoralGrabber1 = SparkMaxConfig()
        configCoralGrabber1.inverted(false)
        configCoralGrabber1.encoder.positionConversionFactor(360.0)
        configCoralGrabber1.encoder.velocityConversionFactor(360.0)
        configCoralGrabber1.smartCurrentLimit(17)
        coralGrabberMotor1.configure(configCoralGrabber1, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    val coralGrabberMotorModel = DCMotor.getNEO(1).withReduction(1.0)
    val coralGrabberMotor1Wrapped = WrappedSparkMax(coralGrabberMotor1, coralGrabberMotorModel)

    val coralGrabberIO = GrabberIO(coralGrabberMotor1Wrapped)

    val coralGrabber = Grabber(
        coralGrabberIO,
        0.0
    )

    val algaeGrabberMotor1 = SparkMax(30, SparkLowLevel.MotorType.kBrushless)

    init {
        var configAlgaeGrabber1 = SparkMaxConfig()
        configAlgaeGrabber1.inverted(false)
        configAlgaeGrabber1.encoder.positionConversionFactor(360.0)
        configAlgaeGrabber1.encoder.velocityConversionFactor(360.0)
        configAlgaeGrabber1.smartCurrentLimit(20)
        algaeGrabberMotor1.configure(configAlgaeGrabber1, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    val algaeGrabberMotorModel = DCMotor.getNEO(1).withReduction(1.0)
    val algaeGrabberMotor1Wrapped = WrappedSparkMax(algaeGrabberMotor1, algaeGrabberMotorModel)

    val algaeGrabberIO = GrabberIO(algaeGrabberMotor1Wrapped)

    val algaeGrabber = Grabber(
        algaeGrabberIO,
        0.0
    )

    val superstructure = Superstructure(elevator, arm, twist)

    init {
        xbox.povUp(loop).rising().ifHigh {
            elevator.goToHeightCommand(elevator.getCurrentHeight() + 0.2, true).schedule()
        }
        xbox.povDown(loop).rising().ifHigh {
            elevator.goToHeightCommand(elevator.getCurrentHeight() - 0.2, false).schedule()
        }
        xbox.povLeft(loop).rising().ifHigh {
            arm.goToAngleCommand(arm.getCurrentAngle() + 15.0, true).schedule()
        }
        xbox.povRight(loop).rising().ifHigh {
            arm.goToAngleCommand(arm.getCurrentAngle() - 15.0, false).schedule()
        }
        buttonBoard.button(1, loop).rising().ifHigh {
            if (buttonBoard.getRawButton(12)) {
                superstructure.goToAlgaeStowedCommand().schedule()
            } else {
                superstructure.goToReefLevelCommand(4)?.schedule()
            }
        }
        buttonBoard.button(2, loop).rising().ifHigh {
            if (buttonBoard.getRawButton(12)) {
                superstructure.goToProcessorCommand().schedule()
            }
            else {
                superstructure.goToReefLevelCommand(3)?.schedule()
            }
        }
        buttonBoard.button(3, loop).rising().ifHigh {
            if (buttonBoard.getRawButton(12)) {
                superstructure.goToReefAlgaeLevelCommand(2)?.schedule()
            } else {
                superstructure.goToReefLevelCommand(2)?.schedule()
            }
        }
        buttonBoard.button(4, loop).rising().ifHigh {
            if (buttonBoard.getRawButton(12)) {
                superstructure.goToReefAlgaeLevelCommand(1)?.schedule()
            } else {
                superstructure.goToReefLevelCommand(1)?.schedule()
            }
        }
        /*buttonBoard.button(5, loop).rising().ifHigh {
        }
        buttonBoard.button(6, loop).rising().ifHigh {
        }*/
        buttonBoard.button(7, loop).rising().ifHigh {
            coralGrabber.outputCommand(6.0).onlyWhile { buttonBoard.getRawButton(7) }.schedule()
        }
        buttonBoard.button(8, loop).rising().ifHigh() {
            coralGrabber.intakeCommand(6.0).onlyWhile { buttonBoard.getRawButton(8) }.schedule()
        }
        buttonBoard.button(9, loop).rising().ifHigh {
            algaeGrabber.intakeCommand(6.0).onlyWhile { buttonBoard.getRawButton(9) }.schedule()
        }
        buttonBoard.button(10, loop).rising().ifHigh {
            algaeGrabber.outputCommand(6.0).onlyWhile { buttonBoard.getRawButton(10) }.schedule()
        }
        buttonBoard.button(11, loop).rising().ifHigh {
            superstructure.goToStationLoadCommand().schedule()
        }
        /*buttonBoard.button(12, loop).rising().ifHigh {
        }*/
        buttonBoard.button(13, loop).rising().ifHigh {
           superstructure.goToStowedCommand().schedule()
        }
    }
}