package net.tecdroid.subsystems.climber

import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.util.*
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
    absoluteEncoderPort = NumericId(2),
    absoluteEncoderOffset = 0.0.rotations,
    absoluteEncoderInverted = false,
    currentLimit = 40.0.amps,
    positiveRotationDirection = Counterclockwise,
    absoluteMaximumAngle = 0.0.rotations,
    absoluteMinimumAngle = 0.0.rotations,
    maximumAngle = 0.0.rotations,
    minimumAngle = 0.0.rotations,
    gearRatio = Reduction(288.0),
    controlGains = ControlGains(),
    motionTargets = AngularMotionTargets(
        cruiseVelocity = 0.1.rotations.per(Second),
        accelerationTimePeriod =  1.0.seconds,
        jerkTimePeriod = 0.1.seconds
    )
)
