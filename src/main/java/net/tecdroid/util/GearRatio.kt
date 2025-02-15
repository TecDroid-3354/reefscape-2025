package net.tecdroid.util

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit

class GearRatio(numerator: Double, denominator: Double) {
    val ratio: Double = numerator / denominator

    fun apply(value: Double): Double {
        return value / ratio
    }

    fun unapply(value: Double): Double {
        return value * ratio
    }

    // TODO: Test on active robot code
    @Suppress("unchecked_cast")
    fun <M : Measure<out Unit?>?> apply(measure: M): M {
        return measure!!.div(ratio) as M
    }

    // TODO: Test on active robot code
    @Suppress("unchecked_cast")
    fun <M : Measure<out Unit?>?> unapply(measure: M): M {
        return measure!!.times(ratio) as M
    }
}
