package net.tecdroid.util.units

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance

fun clamp(min: Angle, max: Angle, value: Angle) : Angle {
    return if (value > max) max else if (value < min) min else value
}

fun clamp(min: Distance, max: Distance, value: Distance) : Distance {
    return if (value > max) max else if (value < min) min else value
}
