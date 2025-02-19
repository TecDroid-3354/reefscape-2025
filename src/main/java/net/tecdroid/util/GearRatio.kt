package net.tecdroid.util

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit

class GearRatio(numerator: Double, denominator: Double, private val gearCount: Int = 0) {
    val ratio: Double = numerator / denominator

    fun apply(value: Double): Double = value / ratio
    fun unapply(value: Double): Double = value * ratio

    fun transformRotation(inputDirection: RotationalDirection) =
        if (gearCount == 0 || gearCount % 2 == 1) inputDirection
        else inputDirection.opposite()

    @Suppress("unchecked_cast")
    fun <M : Measure<out Unit>> apply(measure: M): M = measure.div(ratio) as M

    @Suppress("unchecked_cast")
    fun <M : Measure<out Unit>> unapply(measure: M): M = measure.times(ratio) as M
}
