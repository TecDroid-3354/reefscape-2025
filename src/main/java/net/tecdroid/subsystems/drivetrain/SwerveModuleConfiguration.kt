 package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.util.units.rotations
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.*
import net.tecdroid.util.geometry.Wheel

data class SwerveModuleConfig(
    val driveMotorProperties: MotorProperties,
    val steerMotorProperties: MotorProperties,
    val driveControllerId: NumericId,
    val steerControllerId: NumericId,
    val absoluteEncoderId: NumericId,
    val driveGearRatio: Reduction,
    val steerGearRatio: Reduction,
    val wheel: Wheel,
    val drivePositiveDirection: RotationalDirection,
    val steerPositiveDirection: RotationalDirection,
    val absoluteEncoderMagnetOffset: Angle,
    val driveCurrentLimit: Current,
    val steerCurrentLimit: Current,
    val driveControlGains: ControlGains,
    val steerControlGains: ControlGains
)

fun makeConfig(moduleNumber: DigitId, magnetOffset: Angle) = SwerveModuleConfig(
    driveMotorProperties = Motors.krakenX60,
    steerMotorProperties = Motors.neo,
    driveControllerId = joinDigits(moduleNumber, DigitId(1)),
    steerControllerId = joinDigits(moduleNumber, DigitId(2)),
    absoluteEncoderId = joinDigits(moduleNumber, DigitId(3)),
    driveGearRatio =  Reduction(6.12),
    steerGearRatio = Reduction(150.0 / 7.0),
    wheel =  Wheel.fromRadius(Inches.of(2.0)),
    drivePositiveDirection = Clockwise,
    steerPositiveDirection = Clockwise,
    absoluteEncoderMagnetOffset = magnetOffset,
    driveCurrentLimit = Amps.of(40.0),
    steerCurrentLimit = Amps.of(30.0),
    driveControlGains = ControlGains(s = 0.132, v = 0.12, a = 0.01),
    steerControlGains = ControlGains(p = 0.1, d = 0.01)
)

val frontRightModuleConfig= makeConfig(DigitId(1), (-0.09130859375).rotations)
val frontLeftModuleConfig = makeConfig(DigitId(2), (-0.38982578125).rotations)
val backLeftModuleConfig  = makeConfig(DigitId(3), (-0.345458984375).rotations)
val backRightModuleConfig = makeConfig(DigitId(4), (+0.138427734375).rotations)
