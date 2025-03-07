package net.tecdroid.util

data class PidfCoefficients(val p: Double = 0.0, val i: Double = 0.0, val d: Double = 0.0, val f: Double = 0.0)
data class SvagGains(val s: Double = 0.0, val v: Double = 0.0, val a: Double = 0.0, val g: Double = 0.0)
data class MotionMagicCoefficients(val CruiseVelocity: Double = 0.0, val Acceleration: Double = 0.0, val Jerk: Double = 0.0)