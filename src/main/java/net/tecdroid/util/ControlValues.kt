package net.tecdroid.util

import edu.wpi.first.units.AngularAccelerationUnit
import edu.wpi.first.units.LinearAccelerationUnit
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*

data class PidfCoefficients(val p: Double = 0.0, val i: Double = 0.0, val d: Double = 0.0, val f: Double = 0.0)
data class SvagGains(val s: Double = 0.0, val v: Double = 0.0, val a: Double = 0.0, val g: Double = 0.0)

data class ControlGains(
    val p: Double = 0.0,
    val i: Double = 0.0,
    val d: Double = 0.0,
    val f: Double = 0.0,
    val s: Double = 0.0,
    val v: Double = 0.0,
    val a: Double = 0.0,
    val g: Double = 0.0
)

data class MotionTargets(val cruiseVelocity: Double = 0.0, val acceleration: Double = 0.0, val jerk: Double = 0.0)

data class LinearMotionTargets(
    val cruiseVelocity: LinearVelocity = MetersPerSecond.of(0.0),
    val acceleration: LinearAcceleration = MetersPerSecondPerSecond.of(0.0),
    val jerk: Velocity<LinearAccelerationUnit> = MetersPerSecondPerSecond.of(0.0).per(Second)
)

data class AngularMotionTargets(
    val cruiseVelocity: AngularVelocity = RadiansPerSecond.of(0.0),
    val accelerationTimePeriod: Time,
    val jerkTimePeriod: Time,
    val acceleration: AngularAcceleration = cruiseVelocity.div(accelerationTimePeriod),
    val jerk: Velocity<AngularAccelerationUnit> = acceleration.div(jerkTimePeriod)
)
