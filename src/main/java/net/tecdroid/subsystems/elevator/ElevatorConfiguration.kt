package net.tecdroid.subsystems.elevator

import edu.wpi.first.units.DistanceUnit
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Current
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Clockwise
import net.tecdroid.util.geometry.Sprocket
import net.tecdroid.util.units.amps
import net.tecdroid.util.units.meters
import net.tecdroid.util.units.seconds

data class ElevatorConfig(
    val leadMotorControllerId: NumericId,
    val followerMotorId: NumericId,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,

    val reduction: Reduction,
    val sprocket: Sprocket,

    val limits: SubsystemLimits<DistanceUnit>,
    val controlGains: ControlGains,
    val motionTargets: LinearMotionTargets,
)

val elevatorConfig = ElevatorConfig(
    leadMotorControllerId = NumericId(53),
    followerMotorId = NumericId(54),
    motorDirection = Clockwise,
    motorCurrentLimit = 40.0.amps,

    reduction = Reduction(8.9285),
    sprocket = Sprocket.fromRadius(Inches.of(1 + 1.0 / 8.0)),

    limits = SubsystemLimits(
        absoluteMinimum = 0.0.meters,
        relativeMinimum = 0.02.meters + 0.04.meters,
        relativeMaximum = 1.009.meters - 0.05.meters,
        absoluteMaximum = 1.0389.meters,
    ),

    controlGains = ControlGains(
        p = 0.2,
        s = 0.079404,
        v = 0.1174,
        a = 0.0020923,
        g = 0.23015
    ),

    motionTargets = LinearMotionTargets(
        cruiseVelocity = 1.2.meters.per(Second),
        accelerationTimePeriod = 0.5.seconds,
        jerkTimePeriod = 0.1.seconds
    ),
)
