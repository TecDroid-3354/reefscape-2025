package net.tecdroid.subsystems.intake

import edu.wpi.first.units.measure.Current
import net.tecdroid.util.amps
import net.tecdroid.util.*

data class IntakeConfig(
    val motorControllerId: NumericId,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,
)

public val intakeConfig = IntakeConfig(
    motorControllerId = NumericId(62),
    motorDirection = RotationalDirection.Clockwise,
    motorCurrentLimit = 30.0.amps,
)
