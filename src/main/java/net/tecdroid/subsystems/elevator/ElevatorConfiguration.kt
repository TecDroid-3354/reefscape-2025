package net.tecdroid.subsystems.elevator

import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Clockwise
import net.tecdroid.util.geometry.Sprocket
import net.tecdroid.util.units.amps
import net.tecdroid.util.units.meters
import net.tecdroid.util.units.seconds

data class ElevatorConfig(
    val leadMotorControllerId: NumericId,
    val followerMotorId: NumericId,
    val positiveDirection: RotationalDirection,
    val currentLimit: Current,
    val gearRatio: Reduction,
    val sprocket: Sprocket,
    val absoluteMinimumDisplacement: Distance,
    val absoluteMaximumDisplacement: Distance,
    val minimumDisplacement: Distance,
    val maximumDisplacement: Distance,
    val controlGains: ControlGains,
    val motionTargets: LinearMotionTargets,
)

val elevatorConfig = ElevatorConfig(
    leadMotorControllerId = NumericId(53),
    followerMotorId = NumericId(54),
    positiveDirection = Clockwise,
    currentLimit = 40.0.amps,
    gearRatio = Reduction(8.9285),
    sprocket = Sprocket.fromRadius(Inches.of(1 + 1.0 / 8.0)),
    absoluteMinimumDisplacement = 0.0.meters,
    absoluteMaximumDisplacement = 1.02123.meters,
    minimumDisplacement = 0.06.meters,
    maximumDisplacement = 0.94.meters,
    controlGains = ControlGains(
        p = 0.2,
        s = 0.056261,
        v = 0.11846,
        a = 0.0023818,
        g = 0.24064
    ),
    motionTargets = LinearMotionTargets(0.8.meters.per(Second), 0.5.seconds, 0.1.seconds),
)
