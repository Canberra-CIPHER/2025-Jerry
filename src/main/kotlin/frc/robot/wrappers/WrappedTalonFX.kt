package frc.robot.wrappers

import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.sim.SparkMaxSim
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.wpilibj.RobotBase

class WrappedTalonFX(val talon: TalonFX, val ratio: Double) : VoltageController, PositionProvider {

    override fun setVoltage(voltage: Double) {
        talon.setVoltage(voltage)
    }

    override fun getVoltage(): Double {
        return talon.motorVoltage.valueAsDouble
    }

    override fun setPosition(position: Double) {
        talon.setPosition(position * ratio)
    }

    override fun getPosition(): Double {
        return talon.position.valueAsDouble / ratio
    }

    override fun getCurrent(): Double {
        return talon.supplyCurrent.valueAsDouble
    }
}
