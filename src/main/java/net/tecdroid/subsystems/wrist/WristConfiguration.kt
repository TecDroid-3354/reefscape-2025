package net.tecdroid.subsystems.wrist

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.kt.amps
import net.tecdroid.kt.degrees
import net.tecdroid.kt.rotations
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Clockwise

data class WristConfig(
    val motorProperties: MotorProperties,
    val absoluteEncoderIsInverted: Boolean,
    val motorControllerId: NumericId,
    val absoluteEncoderPort: NumericId,
    val motorCurrentLimit: Current,
    val absoluteMinimumAngle: Angle,
    val absoluteMaximumAngle: Angle,
    val minimumAngle: Angle,
    val maximumAngle: Angle,
    val positiveDirection: RotationalDirection,
    val gearRatio: GearRatio,
    val encoderOffset: Angle,
    val controlGains: ControlGains,
    val motionMagicTargets: MotionMagicTargets
)

val wristConfig = WristConfig(
    motorProperties = Motors.krakenX60,
    absoluteEncoderIsInverted = false,
    motorControllerId = NumericId(61),
    absoluteEncoderPort = NumericId(2),
    motorCurrentLimit = 30.0.amps,
    absoluteMinimumAngle = 0.0.degrees,
    absoluteMaximumAngle = 0.37.rotations,
    minimumAngle = 0.0271.rotations,
    maximumAngle = 0.33.rotations,
    positiveDirection = Clockwise,
    gearRatio = GearRatio(89.2857, 1.0, 0),
    encoderOffset = 0.32876.rotations,
    controlGains = ControlGains(
        s = 0.15,
        g = 0.0
    ),
    motionMagicTargets = MotionMagicTargets()
)
