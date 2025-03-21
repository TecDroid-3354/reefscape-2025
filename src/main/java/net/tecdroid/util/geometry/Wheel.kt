package net.tecdroid.util.geometry

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import java.lang.Math.PI

class Wheel private constructor(radius: Distance) {

    val diameter = radius.times(2.0)
    val circumference = diameter.times(PI)

    fun angularDisplacementToLinearDisplacement(angle: Angle): Distance = circumference.times(angle.`in`(Rotations))
    fun angularVelocityToLinearVelocity(angularVelocity: AngularVelocity): LinearVelocity = circumference.times(angularVelocity.`in`(RotationsPerSecond)).per(Second)

    fun linearDisplacementToAngularDisplacement(distance: Distance): Angle = Rotations.of(distance.div(circumference).baseUnitMagnitude())
    fun linearVelocityToAngularVelocity(linearVelocity: LinearVelocity): AngularVelocity = Rotations.of(linearVelocity.div(circumference).baseUnitMagnitude()).per(Second)

    companion object {
        fun fromRadius(radius: Distance) = Wheel(radius)
    }
}

typealias Sprocket = Wheel