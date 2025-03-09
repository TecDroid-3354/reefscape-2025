package net.tecdroid.subsystems.elevatorjoint

import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.util.units.amps
import net.tecdroid.util.units.rotations
import net.tecdroid.util.*
import net.tecdroid.util.Motors.krakenX60
import net.tecdroid.util.RotationalDirection.Clockwise
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

val elevatorJointConfig = ElevatorJointConfig(
    leadMotorControllerId = NumericId(51),
    followerMotorId = NumericId(52),
    absoluteEncoderPort = NumericId(1),
    absoluteEncoderIsInverted = false,
    motorProperties = krakenX60,
    positiveDirection = Clockwise,
    currentLimit = 40.0.amps,
    gearRatio = GearRatio(150.0, 1.0, 0),
    absoluteMinimumAngle = 0.0.rotations,
    absoluteMaximumAngle = 0.0.rotations,
    minimumAngle = 0.0.rotations,
    maximumAngle = 0.0.rotations,
    controlGains = ControlGains(),
    motionTargets = AngularMotionTargets(0.33.rotations.per(Second), 1.0.seconds,1.0.seconds),
    absoluteEncoderOffset = 0.0.rotations,
)
