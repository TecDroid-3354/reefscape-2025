package net.tecdroid.safety

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit

val pidOutputRange = (-1.0) .. (1.0)

fun <U: Unit> clamp(min: Measure<U>, max: Measure<U>, value: Measure<U>): Measure<U> {
    return if (value > max) max else if (value < min) min else value
}

data class SubsystemLimits <U: Unit>(
    val absoluteMinimum: Measure<U>,
    val relativeMinimum: Measure<U>,
    val relativeMaximum: Measure<U>,
    val absoluteMaximum: Measure<U>,
) {
    init {
        require(absoluteMaximum.gt(relativeMaximum)) { "Absolute Maximum cannot be less than the Relative Maximum" }
        require(absoluteMinimum.lt(relativeMinimum)) { "Absolute Minimum cannot be greater than the Relative Minimum" }
        require(relativeMaximum.gt(relativeMinimum)) { "Relative Maximum cannot be less than the Relative Minimum" }
    }

    fun coerceIn(value: Measure<U>) = clamp(relativeMinimum, relativeMaximum, value)
}