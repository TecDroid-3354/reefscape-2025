package net.tecdroid.subsystems.wrist

import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Time
import net.tecdroid.util.*
import net.tecdroid.util.Motors.krakenX60
import net.tecdroid.util.RotationalDirection.Counterclockwise

data class WristConfig(
    val motorProperties: MotorProperties,
    val absoluteEncoderIsInverted: Boolean,
    val motorControllerId: NumericId,
    val absoluteEncoderPort: NumericId,
    val motorCurrentLimit: Current,
    val maximumAngle: Angle,
    val minimumAngle: Angle,
    val positiveDirection: RotationalDirection,
    val gearRatio: GearRatio,
    val encoderOffset: Angle,
    val controlGains: ControlGains,
    val motionMagicCoefficients: MotionMagicCoefficients
) {
    val allowedAngleMotionRange = minimumAngle .. maximumAngle
}

val wristConfig = WristConfig(
    motorProperties = Motors.krakenX60,
    absoluteEncoderIsInverted = false,
    motorControllerId = NumericId(61),
    absoluteEncoderPort = NumericId(0),
    motorCurrentLimit = Amps.of(30.0),
    maximumAngle = Degrees.of(0.0),
    minimumAngle = Degrees.of(120.0),
    positiveDirection = Counterclockwise,
    gearRatio = GearRatio(89.2857, 1.0, 0),
    encoderOffset = Rotations.of(0.0),
    controlGains = ControlGains(),
    motionMagicCoefficients = MotionMagicCoefficients()
)
