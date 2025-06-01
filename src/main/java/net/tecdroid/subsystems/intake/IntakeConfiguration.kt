package net.tecdroid.subsystems.intake

import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Current
import net.tecdroid.util.amps
import net.tecdroid.util.*

data class IntakeConfig(
    val motorControllerId: NumericId,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,
    val motorNeutralMode: NeutralModeValue
)

public val intakeConfig = IntakeConfig(
    motorControllerId = NumericId(62),
    motorDirection = RotationalDirection.Clockwise,
    motorCurrentLimit = 30.0.amps,
    motorNeutralMode = NeutralModeValue.Brake
)
