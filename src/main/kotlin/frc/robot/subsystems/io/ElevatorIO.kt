package frc.robot.subsystems.io

data class ElevatorIO(
    val getVoltage: () -> Double,
    val setVoltage: (Double) -> Unit,
    val getHeight: () -> Double,
    val setHeight: (Double) -> Unit,
    val getLimitLow: (() -> Boolean)? = null,
    val getLimitHigh: (() -> Boolean)? = null,
    val getCalibrationHeight: (() -> Double)? = null,
)