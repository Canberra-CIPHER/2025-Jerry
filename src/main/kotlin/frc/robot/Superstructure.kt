package frc.robot

import edu.wpi.first.networktables.GenericPublisher
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.RotationSystem
import kotlin.math.absoluteValue

//data class Configuration(val elevatorHeight: Double, val armAngle: Double, val wristAngle: Double)

class Configuration(val name: String, private val initialElevatorHeight: Double, private val initialArmAngle: Double, private val initialWristAngle: Double) {
    private val topicPath = "superstructure/config/${name}"
    private val heightPublisher = NetworkTableInstance.getDefault().getTopic("${topicPath}/height").getGenericEntry("double")
    private val armAnglePublisher = NetworkTableInstance.getDefault().getTopic("${topicPath}/armAngle").getGenericEntry("double")
    private val wristAnglePublisher = NetworkTableInstance.getDefault().getTopic("${topicPath}/wristAngle").getGenericEntry("double")

    init {
        heightPublisher.setDouble(initialElevatorHeight)
        armAnglePublisher.setDouble(initialArmAngle)
        wristAnglePublisher.setDouble(initialWristAngle)
    }

    fun getHeight(): Double {
        return heightPublisher.getDouble(initialElevatorHeight)
    }

    fun getArmAngle(): Double {
        return armAnglePublisher.getDouble(initialArmAngle)
    }

    fun getWristAngle(): Double {
        return wristAnglePublisher.getDouble(initialWristAngle)
    }
}

class Superstructure(private val elevator: Elevator, private val arm: RotationSystem, private val wrist: RotationSystem) {
    private var configurations = emptyMap<String, Configuration>()

    init {
        configurations = mapOf(
            Pair("origin", Configuration("origin", 0.3, 0.0, 0.0)),
            Pair("stowed", Configuration("stowed", 0.0, 0.0, 0.0)),
            Pair("stnLoad", Configuration("stnLoad", 0.15, 38.571, 0.0)),

            Pair("coral1", Configuration("coral1", 0.2, -118.428, 90.0)),
            Pair("coral2", Configuration("coral2", 0.3, -74.429, 90.0)),
            Pair("coral3", Configuration("coral3", 0.672, -49.429, 90.0)),
            Pair("coral4", Configuration("coral4", 1.25, -35.429, 90.0)),

            Pair("algae1", Configuration("algae1", 0.351, 90.0, 90.0)),
            Pair("algae2", Configuration("algae2", 0.9, 90.0, 90.0)),
        )
    }

    fun whenWristIsStowed(): Command {
        return WaitUntilCommand({ -> wrist.getCurrentAngle().absoluteValue < 20.0 })
    }

    fun whenWristOkToTwist(): Command {
        return WaitUntilCommand({ -> arm.getCurrentAngle().absoluteValue > 20.0})
    }

    fun whenArmOkToSwing(): Command {
        return WaitUntilCommand({ -> elevator.getCurrentHeight() > 0.3 })
    }

    fun whenArmIsStowed(): Command {
        return WaitUntilCommand({ -> arm.getCurrentAngle().absoluteValue < 10.0 })
    }

    fun goToOriginCommand(): Command {
        val conf = configurations["origin"]!!
        return elevator.goToHeightCommand(conf.getHeight(), false)
            .andThen(wrist.goToAngleCommand(conf.getWristAngle(), false))
            .andThen(whenWristIsStowed().andThen(arm.goToAngleCommand(conf.getArmAngle(), false)))
    }

    fun goToStowedCommand(): Command {
        val conf = configurations["stowed"]!!

        return goToOriginCommand().andThen(wrist.goToAngleCommand(conf.getWristAngle(), false)
            .alongWith(whenWristIsStowed().andThen(arm.goToAngleCommand(conf.getArmAngle(), false)))
            .alongWith(whenWristIsStowed().andThen(whenArmIsStowed()).andThen(elevator.goToHeightCommand(conf.getHeight(), false)))
        )
        /*return goToOriginCommand().andThen(
            elevator.goToHeightCommand(conf.getHeight(), false)
                .andThen(arm.goToAngleCommand(conf.getArmAngle(), false))
                .andThen(wrist.goToAngleCommand(conf.getWristAngle(), false))
        )*/
    }

    fun goToStationLoadCommand(): Command {
        val conf = configurations["stnLoad"]!!

        return goToOriginCommand().andThen(arm.goToAngleCommand(conf.getArmAngle(), false)).andThen(elevator.goToHeightCommand(conf.getHeight(), false))
            .andThen(wrist.goToAngleCommand(conf.getWristAngle(), false))
    }

    fun goToReefLevelCommand(level: Int): Command {
        val conf = configurations["coral${level}"]!!

        return goToOriginCommand().andThen(elevator.goToHeightCommand(conf.getHeight(), false)
            .alongWith(whenArmOkToSwing().andThen(arm.goToAngleCommand(conf.getArmAngle(), false)))
            .alongWith(wrist.goToAngleCommand(conf.getWristAngle(), false))
        )
        /*return goToOriginCommand().andThen(
            elevator.goToHeightCommand(conf.getHeight(), false)
                .andThen(arm.goToAngleCommand(conf.getArmAngle(), false))
                .andThen(wrist.goToAngleCommand(conf.getWristAngle(), false))
        )*/
    }

    fun goToReefAlgaeLevelCommand(level: Int): Command {
        val conf = configurations["algae${level}"]!!

        return goToOriginCommand().andThen(elevator.goToHeightCommand(conf.getHeight(), false)
            .alongWith(whenArmOkToSwing().andThen(arm.goToAngleCommand(conf.getArmAngle(), false)))
            .alongWith(wrist.goToAngleCommand(conf.getWristAngle(), false))
        )
        /*return goToOriginCommand().andThen(
            elevator.goToHeightCommand(conf.getHeight(), false)
                .andThen(arm.goToAngleCommand(conf.getArmAngle(), false))
                .andThen(wrist.goToAngleCommand(conf.getWristAngle(), false))
        )*/
    }
}