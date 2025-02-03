package frc.robot.subsystems.io

import frc.robot.wrappers.PositionProvider
import frc.robot.wrappers.VoltageController

data class ElevatorIO(
    val voltageController: VoltageController,
    val positionProvider: PositionProvider,
    val getLimitLow: (() -> Boolean)? = null,
    val getLimitHigh: (() -> Boolean)? = null,
    val getCalibrationHeight: (() -> Double?)? = null,
)