package net.tecdroid.subsystems.climber

import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Clockwise
import net.tecdroid.util.RotationalDirection.Counterclockwise
import net.tecdroid.util.units.amps
import net.tecdroid.util.units.rotations
import net.tecdroid.util.units.seconds

data class ClimberConfig(
    val leadingMotorId: NumericId,
    val followerMotorId: NumericId,
    val absoluteEncoderPort: NumericId,
    val absoluteEncoderOffset: Angle,
    val absoluteEncoderInverted: Boolean,
    val currentLimit: Current,
    val positiveRotationDirection: RotationalDirection,
    val absoluteMaximumAngle: Angle,
    val absoluteMinimumAngle: Angle,
    val maximumAngle: Angle,
    val minimumAngle: Angle,
    val gearRatio: Reduction,
    val controlGains: ControlGains,
    val motionTargets: AngularMotionTargets
)

val climberConfig = ClimberConfig(
    leadingMotorId = NumericId(55),
    followerMotorId = NumericId(56),
    absoluteEncoderPort = NumericId(1),
    absoluteEncoderOffset = 0.5415.rotations,
    absoluteEncoderInverted = false,
    currentLimit = 40.0.amps,
    positiveRotationDirection = Clockwise,
    absoluteMaximumAngle = 0.37.rotations,
    absoluteMinimumAngle = 0.0.rotations,
    maximumAngle = 0.335.rotations,
    minimumAngle = 0.031.rotations,
    gearRatio = Reduction(288.0),
    controlGains = ControlGains(
        p = 0.2,
        s = 0.047028,
        v = 0.11345,
        a = 0.001208,
        g = 0.00081703
    ),
    motionTargets = AngularMotionTargets(
        cruiseVelocity = 0.1.rotations.per(Second),
        accelerationTimePeriod =  1.0.seconds,
        jerkTimePeriod = 0.1.seconds
    )
)
