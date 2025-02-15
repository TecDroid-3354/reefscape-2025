package net.tecdroid.util

import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.AngularVelocity

open class MotorProperties(val maxAngularVelocity: AngularVelocity)

object NeoMotor : MotorProperties(RotationsPerSecond.of(94.6))
object KrakenX60: MotorProperties(RotationsPerSecond.of(100.0))
