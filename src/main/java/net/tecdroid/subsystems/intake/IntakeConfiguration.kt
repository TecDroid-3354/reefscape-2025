package net.tecdroid.subsystems.intake

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import net.tecdroid.util.*
import net.tecdroid.util.geometry.Wheel

data class IntakeConfig(
    val motorControllerId: NumericId,
    val beamBreakId: NumericId,
    val motorProperties: MotorProperties,
    val currentLimit: Current,
    val positiveDirection: RotationalDirection,
    val voltageToRetainAlgae: Voltage,
    val controlGains: ControlGains,
    val motionMagicTargets: MotionMagicTargets
)

val intakeConfig = IntakeConfig(
    motorControllerId = NumericId(62),
    beamBreakId = NumericId(3),
    motorProperties = Motors.krakenX60,
    currentLimit = Amps.of(30.0),
    positiveDirection = RotationalDirection.Clockwise,
    voltageToRetainAlgae = Volts.of(0.18),
    controlGains = ControlGains(),
    motionMagicTargets = MotionMagicTargets()
)
