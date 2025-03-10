package net.tecdroid.util.units

import edu.wpi.first.units.measure.Angle

fun clampAngle(min: Angle, max: Angle, value: Angle) : Angle {
    return if (value > max) max else if (value < min) min else value
}