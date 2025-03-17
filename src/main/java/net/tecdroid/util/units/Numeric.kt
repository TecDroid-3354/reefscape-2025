package net.tecdroid.util.units

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit

fun <U: Unit> abs(value: Measure<U>): Measure<U> {
    return if (value.magnitude() < 0) value.unaryMinus() else value
}