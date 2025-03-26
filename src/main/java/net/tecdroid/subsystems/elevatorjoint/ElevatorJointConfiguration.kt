package net.tecdroid.subsystems.elevatorjoint

import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.mechanical.Reduction
import net.tecdroid.safety.MeasureLimits
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Counterclockwise
import net.tecdroid.util.amps
import net.tecdroid.util.rotations
import net.tecdroid.util.seconds

data class ElevatorJointConfig(
    val leadMotorControllerId: NumericId,
    val followerMotorControllerId: NumericId,
    val absoluteEncoderPort: NumericId,
    val absoluteEncoderIsInverted: Boolean,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,
    val reduction: Reduction,
    val measureLimits: MeasureLimits<AngleUnit>,
    val controlGains: ControlGains,
    val motionTargets: AngularMotionTargets,
    val absoluteEncoderOffset: Angle
)

val elevatorJointConfig = ElevatorJointConfig(
    leadMotorControllerId = NumericId(51),
    followerMotorControllerId = NumericId(52),
    motorDirection = Counterclockwise,
    motorCurrentLimit = 40.0.amps,

    absoluteEncoderPort = NumericId(0),
    absoluteEncoderIsInverted = true,
    absoluteEncoderOffset = 0.4840.rotations,

    reduction = Reduction(360.0),

    measureLimits = MeasureLimits(
        absoluteMinimum = 0.011.rotations,
        relativeMinimum = 0.025.rotations,
        relativeMaximum = 0.26.rotations,
        absoluteMaximum = 0.2682.rotations,
    ),

    controlGains = ControlGains(
        p = 0.8,
        s = 0.16263, // 0.14604,
        v = 0.11085, // 0.11017,
        a = 0.002245, // 0.0035221,
        g = 0.01139, // 0.011399
    ),

    motionTargets = AngularMotionTargets(
        cruiseVelocity = 0.2.rotations.per(Second   ),
        accelerationTimePeriod = 0.25.seconds,
        jerkTimePeriod = 0.1.seconds
    )
)
