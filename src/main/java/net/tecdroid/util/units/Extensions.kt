package net.tecdroid.util.units

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*

fun Angle.toRotation2d() = Rotation2d(this)

val Int.pixels
    get() = Pixels.of(this)

val Double.percent
    get() = Percentage(this)

val Double.meters: Distance
    get() = Meters.of(this)

val Double.rotations: Angle
    get() = Rotations.of(this)

val Double.degrees: Angle
    get() = Degrees.of(this)

val Double.radians : Angle
    get() = Radians.of(this)

val Double.seconds: Time
    get() = Seconds.of(this)

val Double.milliseconds: Time
    get() = Milliseconds.of(this)

val Double.volts: Voltage
    get() = Volts.of(this)

val Double.amps: Current
    get() = Amps.of(this)
