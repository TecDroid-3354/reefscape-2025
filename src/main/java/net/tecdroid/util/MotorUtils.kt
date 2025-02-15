package net.tecdroid.util

import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.AngularVelocity

open class MotorProperties(rotationalConvention: RotationalConvention, val maxAngularVelocity: AngularVelocity)

object NeoMotor : MotorProperties(RotationalConvention.counterclockwise(), RotationsPerSecond.of(94.6))
object KrakenX60: MotorProperties(RotationalConvention.counterclockwise(), RotationsPerSecond.of(100.0))
