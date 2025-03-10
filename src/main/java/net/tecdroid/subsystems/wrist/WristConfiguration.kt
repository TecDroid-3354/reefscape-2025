package net.tecdroid.subsystems.wrist

import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.util.units.amps
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.rotations
import net.tecdroid.util.units.seconds
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Clockwise
import net.tecdroid.util.RotationalDirection.Counterclockwise

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
    val absoluteEncoderOffset: Angle,
    val controlGains: ControlGains,
    val motionTargets: AngularMotionTargets
)

public val wristConfig = WristConfig(
    motorProperties = Motors.krakenX60,
    absoluteEncoderIsInverted = false,
    motorControllerId = NumericId(61),
    absoluteEncoderPort = NumericId(2),
    motorCurrentLimit = 30.0.amps,
    absoluteMinimumAngle = 0.0.degrees,
    absoluteMaximumAngle = 0.371.rotations,
    minimumAngle = 0.0271.rotations,
    maximumAngle = 0.33.rotations,
    positiveDirection = Clockwise,
    gearRatio = GearRatio(214.285714, 1.0, 0),
    absoluteEncoderOffset = (0.32761).rotations,
    controlGains = ControlGains(
        p = 0.1,
        s = 0.083806,
        v = 0.11177,
        a = 0.0022073,
        g = 0.0
    ),
    motionTargets = AngularMotionTargets(
        cruiseVelocity = 0.5.rotations.per(Second),
        accelerationTimePeriod = 0.7.seconds,
        jerkTimePeriod = 0.1.seconds
    )
)
