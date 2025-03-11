package net.tecdroid.subsystems.elevatorjoint

import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Counterclockwise
import net.tecdroid.util.units.amps
import net.tecdroid.util.units.rotations
import net.tecdroid.util.units.seconds

data class ElevatorJointConfig(
    val leadMotorControllerId: NumericId,
    val followerMotorId: NumericId,
    val absoluteEncoderPort: NumericId,
    val absoluteEncoderIsInverted: Boolean,
    val positiveDirection: RotationalDirection,
    val currentLimit: Current,
    val reduction: Reduction,
    val limits: SubsystemLimits<AngleUnit>,
    val controlGains: ControlGains,
    val motionTargets: AngularMotionTargets,
    val absoluteEncoderOffset: Angle
)

val elevatorJointConfig = ElevatorJointConfig(
    leadMotorControllerId = NumericId(51),
    followerMotorId = NumericId(52),
    absoluteEncoderPort = NumericId(0),
    absoluteEncoderIsInverted = true,
    positiveDirection = Counterclockwise,
    currentLimit = 40.0.amps,
    reduction = Reduction(360.0),
    limits = SubsystemLimits(
        absoluteMinimum = 0.0.rotations,
        relativeMinimum = 0.018.rotations,
        relativeMaximum = 0.255.rotations,
        absoluteMaximum = 0.2682.rotations,
    ),
    controlGains = ControlGains(
        p = 0.6,
        s = 0.074286, // 0.11123,
        v = 0.11371, // 0.11406,
        a = 0.0014539, // 0.0018075,
        g = 0.0012224, // 0.0064604
    ),
    motionTargets = AngularMotionTargets(0.225.rotations.per(Second), 0.5.seconds, 0.1.seconds),
    absoluteEncoderOffset = 0.4840.rotations,
)
