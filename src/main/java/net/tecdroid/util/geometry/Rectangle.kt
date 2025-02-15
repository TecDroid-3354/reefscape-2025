package net.tecdroid.util.geometry

import edu.wpi.first.units.measure.Distance

open class Rectangle(val length: Distance, val width: Distance) {
    fun scale(scalar: Double) = Rectangle(length * scalar, width * scalar)

}

class Square(sideLength: Distance): Rectangle(sideLength, sideLength)

open class RectangularPrism(val length: Distance, val width: Distance, val height: Distance){
    fun scale(scalar: Double) = RectangularPrism(length * scalar, width * scalar, height * scalar)
}
