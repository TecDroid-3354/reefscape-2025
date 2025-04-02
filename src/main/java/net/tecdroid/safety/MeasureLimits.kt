package net.tecdroid.safety

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit

val pidOutputRange = (-1.0) .. (1.0)

fun <U: Unit> clamp(min: Measure<U>, max: Measure<U>, value: Measure<U>): Measure<U> {
    return if (value > max) max else if (value < min) min else value
}

data class MeasureLimits <U: Unit>(
    val absoluteMinimum: Measure<U>,
    val relativeMinimum: Measure<U>,
    val relativeMaximum: Measure<U>,
    val absoluteMaximum: Measure<U>,
) {
    constructor(absoluteRange: ClosedRange<Measure<U>>, relativeRange: ClosedRange<Measure<U>>) : this(absoluteRange.start, relativeRange.start, relativeRange.endInclusive, absoluteRange.endInclusive)

    init {
        require(absoluteMaximum.gt(relativeMaximum)) { "Absolute Maximum cannot be less than the Relative Maximum" }
        require(absoluteMinimum.lt(relativeMinimum)) { "Absolute Minimum cannot be greater than the Relative Minimum" }
        require(relativeMaximum.gt(relativeMinimum)) { "Relative Maximum cannot be less than the Relative Minimum" }
    }

    fun coerceIn(value: Measure<U>) = clamp(relativeMinimum, relativeMaximum, value)

    operator fun contains(measure: Measure<U>): Boolean {
        return relativeMinimum.lt(measure) && relativeMaximum.gt(measure)
    }

    operator fun compareTo(measure: Measure<U>): Int =
        if (measure in this) 0
        else if (measure > relativeMaximum) -1
        else 1

}

operator fun <U: Unit> Measure<U>.compareTo(measureLimits: MeasureLimits<U>): Int {
    return if (this in measureLimits) 0
    else if (this > measureLimits.relativeMaximum) return 1
    else -1
}
