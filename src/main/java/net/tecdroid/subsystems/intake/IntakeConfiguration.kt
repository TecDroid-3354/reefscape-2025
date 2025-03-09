package net.tecdroid.subsystems.intake

import edu.wpi.first.units.measure.Current
import net.tecdroid.kt.amps
import net.tecdroid.util.*

data class IntakeConfig(
    val motorControllerId: NumericId,
    val motorProperties: MotorProperties,
    val currentLimit: Current,
    val positiveDirection: RotationalDirection,
    val controlGains: ControlGains,
    val motionMagicTargets: MotionMagicTargets
)

val intakeConfig = IntakeConfig(
    motorControllerId = NumericId(62),
    motorProperties = Motors.krakenX60,
    currentLimit = 30.0.amps,
    positiveDirection = RotationalDirection.Clockwise,
    controlGains = ControlGains(),
    motionMagicTargets = MotionMagicTargets()
)
