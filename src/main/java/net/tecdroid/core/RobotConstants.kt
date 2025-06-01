package net.tecdroid.core

/**
 * Enum class to describe if the code is running on a real robot, simulated robot, or in a replay.
 */
enum class RobotMode {
    REAL, SIMULATION, REPLAY
}

/**
 * Object to access the current mode of the robot.
 */
object RobotConstants {
    val currentMode = RobotMode.REAL
}