package net.tecdroid.subsystems.wrist

import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Clockwise
import net.tecdroid.util.units.amps
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.rotations
import net.tecdroid.util.units.seconds
import net.tecdroid.util.SubsystemLimits

data class WristConfig(
    val absoluteEncoderIsInverted: Boolean,
    val motorControllerId: NumericId,
    val absoluteEncoderPort: NumericId,
    val motorCurrentLimit: Current,
    val positiveDirection: RotationalDirection,
    val reduction: Reduction,
    val absoluteEncoderOffset: Angle,
    val limits: SubsystemLimits<AngleUnit>,
    val controlGains: ControlGains,
    val motionTargets: AngularMotionTargets,
)

val wristConfig = WristConfig(
    absoluteEncoderIsInverted = false,
    motorControllerId = NumericId(61),
    absoluteEncoderPort = NumericId(2),
    motorCurrentLimit = 30.0.amps,
    positiveDirection = Clockwise,
    reduction = Reduction(214.285714),
    absoluteEncoderOffset = (0.32761).rotations,

    limits = SubsystemLimits(
        absoluteMinimum = 0.0.degrees,
        absoluteMaximum = 0.371.rotations,
        relativeMinimum = 0.0271.rotations,
        relativeMaximum = 0.33.rotations,
    ),

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