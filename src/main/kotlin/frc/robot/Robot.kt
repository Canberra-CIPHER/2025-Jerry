//Our code for our chiddler Gheoughrobarhgeaux

package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj.TimedRobot
import au.grapplerobotics.GrappleJNI
import au.grapplerobotics.CanBridge
import edu.wpi.first.networktables.NetworkTableInstance

class Robot : TimedRobot() {
    private var autonomousCommand: Command? = null
    private var robotContainer: RobotContainer? = null

    val publisherPitch = NetworkTableInstance.getDefault().getTopic("gyro/pitch").genericPublish("double")
    val publisherYaw = NetworkTableInstance.getDefault().getTopic("gyro/yaw").genericPublish("double")
    val publisherRoll = NetworkTableInstance.getDefault().getTopic("gyro/roll").genericPublish("double")

    override fun robotInit() {
        robotContainer = RobotContainer()
        CanBridge.runTCP()
        CommandScheduler.getInstance().registerSubsystem(robotContainer?.driveSystem!!)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        publisherYaw.setDouble(robotContainer?.gyro?.yaw?.toDouble()!!)
        publisherPitch.setDouble(robotContainer?.gyro?.pitch?.toDouble()!!)
        publisherRoll.setDouble(robotContainer?.gyro?.roll?.toDouble()!!)
        robotContainer?.loop?.poll()
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
        autonomousCommand = robotContainer?.driveSystem?.snapToAngleCommand(45.0, true)
        autonomousCommand?.schedule()
    }

    override fun autonomousPeriodic() {}

    override fun autonomousExit() {}

    override fun teleopInit() {
        autonomousCommand?.cancel()
        CommandScheduler.getInstance().setDefaultCommand(robotContainer?.driveSystem, robotContainer?.driveSystem?.driveDefaultCommand(robotContainer?.xbox!!))
    }

    override fun teleopPeriodic() {}

    override fun teleopExit() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {}

    override fun testExit() {}
}
