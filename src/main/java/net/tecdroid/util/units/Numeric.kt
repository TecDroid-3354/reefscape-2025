package net.tecdroid.util.units

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit

fun <U: Unit> clamp(min: Measure<U>, max: Measure<U>, value: Measure<U>) : Measure<U> {
    return if (value > max) max else if(value < max) min else value
}