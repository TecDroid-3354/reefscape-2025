package net.tecdroid.subsystems.elevatorjoint

import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.util.*
import net.tecdroid.util.Motors.krakenX60
import net.tecdroid.util.RotationalDirection.Counterclockwise
import net.tecdroid.util.units.amps
import net.tecdroid.util.units.rotations
import net.tecdroid.util.units.seconds

data class ElevatorJointConfig(
    val leadMotorControllerId: NumericId,
    val followerMotorId: NumericId,
    val absoluteEncoderPort: NumericId,
    val absoluteEncoderIsInverted: Boolean,
    val motorProperties: MotorProperties,
    val positiveDirection: RotationalDirection,
    val currentLimit: Current,
    val gearRatio: GearRatio,
    val absoluteMinimumAngle: Angle,
    val absoluteMaximumAngle: Angle,
    val minimumAngle: Angle,
    val maximumAngle: Angle,
    val controlGains: ControlGains,
    val motionTargets: AngularMotionTargets,
    val absoluteEncoderOffset: Angle
)

public val elevatorJointConfig = ElevatorJointConfig(
    leadMotorControllerId = NumericId(51),
    followerMotorId = NumericId(52),
    absoluteEncoderPort = NumericId(0),
    absoluteEncoderIsInverted = false,
    motorProperties = krakenX60,
    positiveDirection = Counterclockwise,
    currentLimit = 40.0.amps,
    gearRatio = GearRatio(360.0, 1.0, 0),
    absoluteMinimumAngle = 0.0.rotations,
    absoluteMaximumAngle = 0.26.rotations,
    minimumAngle = 0.014.rotations,
    maximumAngle = 0.25.rotations,
    controlGains = ControlGains(
        p = 0.6,
        s = 0.11123,
        v = 0.11406,
        a = 0.0018075,
        g = 0.0064604
    ),
    motionTargets = AngularMotionTargets(0.5.rotations.per(Second), 0.5.seconds, 0.1.seconds),
    absoluteEncoderOffset = 0.5081.rotations,
)
