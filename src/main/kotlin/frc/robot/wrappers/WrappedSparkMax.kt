package frc.robot.wrappers

import com.revrobotics.sim.SparkMaxSim
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase

class WrappedSparkMax(val sparkMax: SparkMax, val motor: DCMotor) : VoltageController, PositionProvider {
    var sim: SparkMaxSim? = null

    init {
        if (RobotBase.isSimulation()) {
            sim = SparkMaxSim(sparkMax, motor)
        }
    }

    override fun setVoltage(voltage: Double) {
        sparkMax.setVoltage(voltage)
    }

    override fun getVoltage(): Double {
        return sparkMax.busVoltage * sparkMax.appliedOutput
    }

    override fun setPosition(position: Double) {
        sparkMax.encoder.position = position
    }

    override fun getPosition(): Double {
        return sparkMax.encoder.position
    }
}
