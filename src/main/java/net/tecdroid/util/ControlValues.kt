package net.tecdroid.util

import edu.wpi.first.units.AngularAccelerationUnit
import edu.wpi.first.units.LinearAccelerationUnit
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import net.tecdroid.util.geometry.Wheel

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

data class AngularMotionTargets(
    val cruiseVelocity: AngularVelocity = RadiansPerSecond.of(0.0),
    val accelerationTimePeriod: Time,
    val jerkTimePeriod: Time,
    val acceleration: AngularAcceleration = cruiseVelocity.div(accelerationTimePeriod),
    val jerk: Velocity<AngularAccelerationUnit> = acceleration.div(jerkTimePeriod)
)

data class LinearMotionTargets(
    val cruiseVelocity: LinearVelocity = MetersPerSecond.of(0.0),
    val accelerationTimePeriod: Time,
    val jerkTimePeriod: Time,
) {
    val acceleration: LinearAcceleration = cruiseVelocity.div(accelerationTimePeriod)
    val jerk: Velocity<LinearAccelerationUnit> = acceleration.div(jerkTimePeriod)

    fun angularVelocity(wheel: Wheel) = wheel.linearVelocityToAngularVelocity(cruiseVelocity)
    fun angularAcceleration(wheel: Wheel) = angularVelocity(wheel).div(accelerationTimePeriod)
    fun angularJerk(wheel: Wheel) = angularAcceleration(wheel).div(jerkTimePeriod)
}
