package frc.robot.subsystems.io

import frc.robot.wrappers.VoltageController

data class GrabberIO(
    val voltageController: VoltageController,
    val hasGamePiece: (() -> Boolean?)? = null,
)