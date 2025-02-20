package net.tecdroid.util.geometry

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import kotlin.math.sqrt

open class Rectangle(val length: Distance, val width: Distance) {
    fun scale(scalar: Double) = Rectangle(length * scalar, width * scalar)

    val diagonalLengthSquared = width * width + length * length
    val diagonalLength = Meters.baseUnit.of(sqrt(diagonalLengthSquared.baseUnitMagnitude()))

}

class Square(sideLength: Distance) : Rectangle(sideLength, sideLength)
