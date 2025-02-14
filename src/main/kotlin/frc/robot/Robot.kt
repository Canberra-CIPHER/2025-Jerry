//Our code for our chiddler Gheoughrobarhgeaux Bartholomew Barlo-quenné Norman Smith
//AKA Jerry AKA Jerryatrick
//Born under an Aquarius sun, Aries rising, and Scorpio moon, in the year of the Dragon

package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj.TimedRobot
import au.grapplerobotics.CanBridge
import edu.wpi.first.networktables.NetworkTableInstance

class Robot : TimedRobot() {
    private var autonomousCommand: Command? = null
    private var robotContainer: RobotContainer? = null

    override fun robotInit() {
        robotContainer = RobotContainer()
        CanBridge.runTCP()
        //CommandScheduler.getInstance().registerSubsystem(robotContainer?.driveSystem!!)
        CommandScheduler.getInstance().registerSubsystem(robotContainer?.swerveDriveSystem!!)
        //CommandScheduler.getInstance().registerSubsystem(robotContainer?.elevator!!)
        //CommandScheduler.getInstance().registerSubsystem(robotContainer?.arm!!)
//        CommandScheduler.getInstance().registerSubsystem(robotContainer?.twist!!)
//        CommandScheduler.getInstance().registerSubsystem(robotContainer?.wrist!!)
       //CommandScheduler.getInstance().registerSubsystem(robotContainer?.grabber!!)

        //addPeriodic({ -> robotContainer?.elevator?.controlPeriodic() }, 0.01)
        //addPeriodic({ -> robotContainer?.arm?.controlPeriodic() }, 0.01)
//        addPeriodic({ -> robotContainer?.twist?.controlPeriodic() }, 0.01)
//        addPeriodic({ -> robotContainer?.wrist?.controlPeriodic() }, 0.01)
        //addPeriodic({ -> robotContainer?.grabber?.controlPeriodic() }, 0.01)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        robotContainer?.loop?.poll()
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
        /*autonomousCommand = robotContainer?.driveSystem?.snapToAngleCommand(45.0, true)
        autonomousCommand?.schedule()*/
    }

    override fun autonomousPeriodic() {}

    override fun autonomousExit() {}

    override fun teleopInit() {
        autonomousCommand?.cancel()
        //CommandScheduler.getInstance().setDefaultCommand(robotContainer?.driveSystem, robotContainer?.driveSystem?.driveDefaultCommand(robotContainer?.xbox!!))
        CommandScheduler.getInstance().setDefaultCommand(robotContainer?.swerveDriveSystem, robotContainer?.swerveDriveSystem?.driveDefaultCommand(
            { -> robotContainer!!.xbox.leftX },
            { -> robotContainer!!.xbox.leftY},
            { -> robotContainer!!.xbox.rightX },
            { -> robotContainer!!.xbox.rightY },
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
