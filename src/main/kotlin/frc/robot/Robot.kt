//Our code for our chiddler Gheoughrobarhgeaux Bartholomew Barlo-quenné Norman Smith
//AKA Jerry AKA Jerryatrick
//Born under an Aquarius sun, Aries rising, and Scorpio moon, in the year of the Dragon

package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj.TimedRobot
import au.grapplerobotics.CanBridge
import com.fasterxml.jackson.databind.JsonSerializer.None
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.networktables.NetworkTableInstance
import kotlin.math.sign

class Robot : TimedRobot() {
    private var autonomousCommand: Command? = null
    private var robotContainer: RobotContainer? = null

    val elevatorCANPublisher = NetworkTableInstance.getDefault().getTopic("elevator/lasercan").genericPublish("double")

    override fun robotInit() {
        robotContainer = RobotContainer()
        CanBridge.runTCP()
        //CommandScheduler.getInstance().registerSubsystem(robotContainer?.driveSystem!!)
        CommandScheduler.getInstance().registerSubsystem(robotContainer?.swerveDriveSystem!!)
        CommandScheduler.getInstance().registerSubsystem(robotContainer?.elevator!!)
        CommandScheduler.getInstance().registerSubsystem(robotContainer?.arm!!)
        CommandScheduler.getInstance().registerSubsystem(robotContainer?.twist!!)
        CommandScheduler.getInstance().registerSubsystem(robotContainer?.coralGrabber!!)
        CommandScheduler.getInstance().registerSubsystem(robotContainer?.algaeGrabber!!)

        addPeriodic({ -> robotContainer?.swerveDriveSystem?.controlPeriodic() }, 0.01)
        addPeriodic({ -> robotContainer?.elevator?.controlPeriodic() }, 0.01)
        addPeriodic({ -> robotContainer?.arm?.controlPeriodic() }, 0.01)
        addPeriodic({ -> robotContainer?.twist?.controlPeriodic() }, 0.01)
        addPeriodic({ -> robotContainer?.coralGrabber?.controlPeriodic() }, 0.01)
        addPeriodic({ -> robotContainer?.algaeGrabber?.controlPeriodic() }, 0.01)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        robotContainer?.loop?.poll()
        elevatorCANPublisher.setInteger((robotContainer!!.liftLaserCAN?.measurement?.distance_mm ?: 0).toLong())
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
        /*autonomousCommand = robotContainer?.driveSystem?.snapToAngleCommand(45.0, true)
        autonomousCommand?.schedule()*/
        val pose = robotContainer!!.swerveDriveSystem.currentPose()
        robotContainer!!.swerveDriveSystem.driveToPosition(pose.plus(Transform2d(
            0.0, 0.5, Rotation2d.fromDegrees(0.0)
        ))).schedule()
    }

    override fun autonomousPeriodic() {}

    override fun autonomousExit() {}

    fun squareInputs(input: Double): Double {
        return input.sign * input * input
    }

    fun getThrottleMultiplier(): Double {
        if (robotContainer?.xbox!!.rightBumperButton) {
            return 0.35
        }
        else {
            return 0.5
        }
    }

    override fun teleopInit() {
        autonomousCommand?.cancel()

        val snapFun = { ->
            var snapX: Double? = null
            var snapY: Double? = null

            if (robotContainer!!.xbox.xButton) {
                snapX = 0.0
                snapY = -1.0
            }
            else if (robotContainer!!.xbox.aButton) {
                snapX = -1.0
                snapY = 0.0
            }
            else if (robotContainer!!.xbox.bButton) {
                snapX = 0.0
                snapY = 1.0
            }
            else if (robotContainer!!.xbox.yButton) {
                snapX = 1.0
                snapY = 0.0
            }

            Pair(snapX, snapY)
        }

        CommandScheduler.getInstance().setDefaultCommand(robotContainer?.swerveDriveSystem, robotContainer?.swerveDriveSystem?.driveDefaultCommand(
            { -> squareInputs(-robotContainer!!.xbox.leftX) * getThrottleMultiplier() },
            { -> squareInputs(robotContainer!!.xbox.leftY) * getThrottleMultiplier() },
            { -> snapFun.invoke().first ?: -robotContainer!!.xbox.rightX },
            { -> snapFun.invoke().second ?: -robotContainer!!.xbox.rightY },
        ))
}

override fun teleopPeriodic() {}

override fun teleopExit() {}

override fun testInit() {
    CommandScheduler.getInstance().cancelAll()
}

override fun testPeriodic() {}

override fun testExit() {}
}
