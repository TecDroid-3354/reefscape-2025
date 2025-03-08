package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.kt.rotations
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Counterclockwise
import net.tecdroid.util.geometry.Wheel

data class SwerveModuleConfig(
    val driveMotorProperties: MotorProperties,
    val steerMotorProperties: MotorProperties,
    val driveControllerId: NumericId,
    val steerControllerId: NumericId,
    val absoluteEncoderId: NumericId,
    val driveGearRatio: GearRatio,
    val steerGearRatio: GearRatio,
    val wheel: Wheel,
    val drivePositiveDirection: RotationalDirection,
    val steerPositiveDirection: RotationalDirection,
    val absoluteEncoderMagnetOffset: Angle,
    val driveCurrentLimit: Current,
    val steerCurrentLimit: Current,
    val driveControlGains: ControlGains,
    val steerControlGains: ControlGains
)

private val driveGearRatio = GearRatio(6.12, 1.0, 4)
private val steerGearRatio = GearRatio(150.0, 7.0, 2)
private val wheel = Wheel.fromRadius(Inches.of(2.0))

private val driveWheelPositiveDirection = Counterclockwise;
private val steerAzimuthPositiveDirection = Counterclockwise;

private val driveMotorProperties = Motors.krakenX60
private val steerMotorProperties = Motors.neo
private val driveMotorCurrentLimit: Current = Amps.of(40.0)
private val steerMotorCurrentLimit: Current = Amps.of(30.0)

private val driveControlGains = ControlGains(s = 0.132, v = 0.12, a = 0.01)
private val steerControlGains = ControlGains(p = 0.1, d = 0.01)

fun makeConfig(moduleNumber: DigitId, magnetOffset: Angle) = SwerveModuleConfig(
    driveMotorProperties = driveMotorProperties,
    steerMotorProperties = steerMotorProperties,
    driveControllerId = joinDigits(moduleNumber, DigitId(1)),
    steerControllerId = joinDigits(moduleNumber, DigitId(2)),
    absoluteEncoderId = joinDigits(moduleNumber, DigitId(3)),
    driveGearRatio = driveGearRatio,
    steerGearRatio = steerGearRatio,
    wheel = wheel,
    drivePositiveDirection = driveWheelPositiveDirection,
    steerPositiveDirection =   steerAzimuthPositiveDirection,
    absoluteEncoderMagnetOffset = magnetOffset,
    driveCurrentLimit = driveMotorCurrentLimit,
    steerCurrentLimit = steerMotorCurrentLimit,
    driveControlGains = driveControlGains,
    steerControlGains = steerControlGains
)

val frontRightModuleConfig= makeConfig(DigitId(1), (-0.09130859375).rotations)
val frontLeftModuleConfig = makeConfig(DigitId(2), (-0.38982578125).rotations)
val backLeftModuleConfig  = makeConfig(DigitId(3), (-0.345458984375).rotations)
val backRightModuleConfig = makeConfig(DigitId(4), (+0.138427734375).rotations)
