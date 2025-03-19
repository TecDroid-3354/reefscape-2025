package net.tecdroid.subsystems.wrist

import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.mechanical.Reduction
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Clockwise
import net.tecdroid.util.units.amps
import net.tecdroid.util.units.rotations
import net.tecdroid.util.units.seconds
import net.tecdroid.safety.SubsystemLimits

data class WristConfig(
    val motorControllerId: NumericId,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,

    val absoluteEncoderPort: NumericId,
    val absoluteEncoderIsInverted: Boolean,
    val absoluteEncoderOffset: Angle,

    val reduction: Reduction,
    val limits: SubsystemLimits<AngleUnit>,
    val controlGains: ControlGains,
    val motionTargets: AngularMotionTargets,
)

val wristConfig = WristConfig(
    motorControllerId = NumericId(61),
    motorDirection = Clockwise,
    motorCurrentLimit = 30.0.amps,

    absoluteEncoderPort = NumericId(2),
    absoluteEncoderIsInverted = false,
    absoluteEncoderOffset = (0.1501).rotations,

    reduction = Reduction(214.285714),

    limits = SubsystemLimits(
        absoluteMinimum = 0.0.rotations,
        relativeMinimum = 0.021.rotations,
        relativeMaximum = 0.3704.rotations,
        absoluteMaximum = 0.3748.rotations,
    ),

    controlGains = ControlGains(
        p = 0.1,
        s = 0.1191,
        v = 0.11037,
        a = 0.0019686,
        g = 0.0012195
    ),

    motionTargets = AngularMotionTargets(
        cruiseVelocity = 0.5.rotations.per(Second),
        accelerationTimePeriod = 0.5.seconds,
        jerkTimePeriod = 0.1.seconds
    )
)