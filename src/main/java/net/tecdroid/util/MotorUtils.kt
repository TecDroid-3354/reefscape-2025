package net.tecdroid.util

import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.AngularVelocity

/**
 * Allows grouping the properties of a motor
 */
data class MotorProperties(val positiveDirection: RotationalDirection, val maxAngularVelocity: AngularVelocity)

/**
 * Contains an assortment FRC legal motor properties
 */
object Motors {
    val neo = MotorProperties(RotationalDirection.Counterclockwise, RotationsPerSecond.of(94.6))
    val krakenX60 = MotorProperties(RotationalDirection.Counterclockwise, RotationsPerSecond.of(100.0))
}
