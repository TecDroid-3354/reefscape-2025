 package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.mechanical.Reduction
import net.tecdroid.util.rotations
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.*
import net.tecdroid.util.Circle

data class SwerveModuleConfig(
    val driveMotorProperties: MotorProperties,
    val steerMotorProperties: MotorProperties,
    val driveControllerId: NumericId,
    val steerControllerId: NumericId,
    val absoluteEncoderId: NumericId,
    val driveGearRatio: Reduction,
    val steerGearRatio: Reduction,
    val circle: Circle,
    val drivePositiveDirection: RotationalDirection,
    val steerPositiveDirection: RotationalDirection,
    val absoluteEncoderMagnetOffset: Angle,
    val driveCurrentLimit: Current,
    val steerCurrentLimit: Current,
    val driveControlGains: ControlGains,
    val steerControlGains: ControlGains
)

fun makeConfig(moduleNumber: SymbolicId, magnetOffset: Angle, controlGains: ControlGains) = SwerveModuleConfig(
    driveMotorProperties = Motors.krakenX60,
    steerMotorProperties = Motors.neo,
    driveControllerId = moduleNumber * 10 + NumericId(1),
    steerControllerId = moduleNumber * 10 + NumericId(2),
    absoluteEncoderId = moduleNumber * 10 + NumericId(3),
    driveGearRatio =  Reduction(6.12),
    steerGearRatio = Reduction(150.0 / 7.0),
    circle =  Circle.fromRadius(Inches.of(2.0)),
    drivePositiveDirection = Clockwise,
    steerPositiveDirection = Clockwise,
    absoluteEncoderMagnetOffset = magnetOffset,
    driveCurrentLimit = Amps.of(40.0),
    steerCurrentLimit = Amps.of(30.0),
    driveControlGains = controlGains,
    steerControlGains = ControlGains(p = 0.35)
)

val frontRightModuleConfig= makeConfig(SymbolicId(1), (-0.098876953125).rotations, ControlGains(s = 0.18174, v = 0.11491, a = 0.005462))
val frontLeftModuleConfig = makeConfig(SymbolicId(2), (-0.390380859375).rotations, ControlGains(s = 0.12853, v = 0.11318, a = 0.0058254))
val backLeftModuleConfig  = makeConfig(SymbolicId(3), (-0.342041015625).rotations, ControlGains(s = 0.1435, v = 0.11366, a = 0.0091299))
val backRightModuleConfig = makeConfig(SymbolicId(4), (+0.13574211875).rotations, ControlGains(s = 0.21496, v = 0.11182, a = 0.008843))
