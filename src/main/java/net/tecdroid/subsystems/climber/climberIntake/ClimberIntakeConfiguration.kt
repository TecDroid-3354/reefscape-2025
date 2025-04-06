package net.tecdroid.subsystems.climber.climberIntake

import edu.wpi.first.units.measure.Current
import net.tecdroid.util.NumericId
import net.tecdroid.util.RotationalDirection
import net.tecdroid.util.amps

data class ClimberIntakeConfig(
    val motorControllerId: NumericId,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,
)

public val climberIntakeConfig = ClimberIntakeConfig(
    motorControllerId = NumericId(92), // TODO: CHECK THE NUMERIC ID
    motorDirection = RotationalDirection.Clockwise, // TODO: CHECK DIRECTION
    motorCurrentLimit = 30.0.amps,
)
