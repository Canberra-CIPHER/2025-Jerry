package frc.robot.wrappers

interface VoltageController {
    fun setVoltage(voltage: Double)
    fun getVoltage(): Double
    fun getCurrent(): Double
}