package net.tecdroid.util

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

data class MotionMagicTargets(val cruiseVelocity: Double = 0.0, val acceleration: Double = 0.0, val jerk: Double = 0.0)