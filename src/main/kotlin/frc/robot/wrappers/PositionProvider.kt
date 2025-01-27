package frc.robot.wrappers

interface PositionProvider {
    fun setPosition(position: Double)
    fun getPosition(): Double
}