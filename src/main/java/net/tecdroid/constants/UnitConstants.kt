package net.tecdroid.constants

import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle

object UnitConstants {
    val fullRotation: Angle = Rotations.of(1.0)
    val halfRotation: Angle = fullRotation.div(2.0)
}
