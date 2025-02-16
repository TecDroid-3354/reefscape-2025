package net.tecdroid.util

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import net.tecdroid.util.LongitudinalDirection.*
import net.tecdroid.util.RotationalDirection.*
import net.tecdroid.util.TransversalDirection.Left
import net.tecdroid.util.TransversalDirection.Right
import net.tecdroid.util.VerticalDirection.Down
import net.tecdroid.util.VerticalDirection.Up

enum class RotationalDirection {
    Clockwise, Counterclockwise;

    fun isClockwise() = this == Clockwise
    fun isCounterClockwise() = this == Clockwise

    fun matches(other: RotationalDirection) = this == other
    fun differs(other: RotationalDirection) = !matches(other)

    fun toSensorDirectionValue() = if (this == Clockwise) SensorDirectionValue.Clockwise_Positive else SensorDirectionValue.CounterClockwise_Positive
    fun toInvertedValue() = if (this == Clockwise) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
}

class RotationalConvention(val direction: RotationalDirection) {
    fun clockwise(angle: Angle): Angle = if (direction == Clockwise) angle else angle.times(-1.0)
    fun counterclockwise(angle: Angle): Angle = if (direction == Counterclockwise) angle else angle.times(-1.0)
    fun convertTo(other: RotationalConvention, angle: Angle) = if (this.direction == other.direction) angle else angle.times(-1.0)

    companion object {
        fun clockwisePositive() = RotationalConvention(Clockwise)
        fun counterclockwisePositive() = RotationalConvention(Counterclockwise)
    }
}

enum class LongitudinalDirection{
    Front, Back;

    fun matches(other: LongitudinalDirection) = this == other
    fun differs(other: LongitudinalDirection) = !matches(other)

    fun isFront() = this == Front
    fun isBack() = this == Back
}

class LongitudinalConvention(val direction: LongitudinalDirection) {
    fun front(distance: Distance) = if (direction == Front) distance else distance.times(-1.0)
    fun back(distance: Distance) = if (direction == Back) distance else distance.times(-1.0)
    fun convertTo(other: LongitudinalConvention, distance: Distance) = if (this.direction == other.direction) distance else distance.times(-1.0)

    companion object {
        fun frontPositive() = LongitudinalConvention(Front)
        fun backPositive() = LongitudinalConvention(Back)
    }
}

enum class TransversalDirection {
    Left, Right;

    fun matches(other: TransversalDirection) = this == other
    fun differs(other: TransversalDirection) = !matches(other)

    fun isLeft() = this == Left
    fun isRight() = this == Right
}

class TransversalConvention(val direction: TransversalDirection) {
    fun left(distance: Distance) = if (direction == Left) distance else distance.times(-1.0)
    fun right(distance: Distance) = if (direction == Right) distance else distance.times(-1.0)
    fun convertTo(other: TransversalConvention, distance: Distance) = if (this.direction == other.direction) distance else distance.times(-1.0)

    companion object {
        fun leftPositive() = TransversalConvention(Left)
        fun rightPositive() = TransversalConvention(Right)
    }
}

enum class VerticalDirection{
    Up, Down;

    fun matches(other: VerticalDirection) = this == other
    fun differs(other: VerticalDirection) = !matches(other)

    fun isUp() = this == Up
    fun isDown() = this == Down
}

class VerticalConvention(val direction: VerticalDirection) {
    fun up(distance: Distance) = if (direction == Up) distance else distance.times(-1.0)
    fun down(distance: Distance) = if (direction == Down) distance else distance.times(-1.0)
    fun convertTo(other: VerticalConvention, distance: Distance) = if (this.direction == other.direction) distance else distance.times(-1.0)

    companion object {
        fun up() = VerticalConvention(Up)
        fun down() = VerticalConvention(Down)
    }
}

open class SpatialConvention(
    val longitudinal: LongitudinalConvention,
    val transversal: TransversalConvention,
    val rotational: VerticalConvention,
    val rotationalConvention: RotationalConvention)

class SpatialConversion(private val from: SpatialConvention, private val to: SpatialConvention) {
    fun convertLongitudinal(distance: Distance) = from.longitudinal.convertTo(
        to.longitudinal,
        distance
    )
    fun convertTransversal(distance: Distance) = from.transversal.convertTo(
        to.transversal,
        distance
    )
    fun convertVertical(distance: Distance) = from.rotational.convertTo(to.rotational, distance)
    fun convertRotational(angle: Angle) = from.rotationalConvention.convertTo(to.rotationalConvention, angle)

}
