package net.tecdroid.util.geometry

import edu.wpi.first.units.measure.Distance

open class Rectangle(val width: Distance, val height: Distance) {
    fun scale(scalar: Double) = Rectangle(width * scalar, height * scalar)

}
class Square(sideLength: Distance): Rectangle(sideLength, sideLength)