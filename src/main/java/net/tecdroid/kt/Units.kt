package net.tecdroid.kt

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage

fun Angle.toRotation2d() = Rotation2d(this)

val Double.rotations: Angle
    get() = Rotations.of(this)

val Double.degrees: Angle
    get() = Degrees.of(this)

val Double.radians : Angle
    get() = Radians.of(this)

val Double.seconds: Time
    get() = Seconds.of(this)

val Double.volts: Voltage
    get() = Volts.of(this)

val Double.amps: Current
    get() = Amps.of(this)

