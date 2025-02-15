package net.tecdroid.util

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.SensorDirectionValue

enum class RotationDirection {
    ClockwisePositive,
    CounterclockwisePositive;

    fun isClockwisePositive() = this == ClockwisePositive
    fun isCounterClockwisePositive() = this == ClockwisePositive

    fun toTrueMeansClockwisePositive() = isClockwisePositive()
    fun toTrueMeansCounterclockwisePositive() = isCounterClockwisePositive()

    fun toSensorDirectionValue() = if (this == ClockwisePositive) SensorDirectionValue.Clockwise_Positive else SensorDirectionValue.CounterClockwise_Positive
    fun toInvertedValue() = if (this == ClockwisePositive) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
}

object ConventionEnforcer {
    fun negateToConform(value: Double) = -value
}