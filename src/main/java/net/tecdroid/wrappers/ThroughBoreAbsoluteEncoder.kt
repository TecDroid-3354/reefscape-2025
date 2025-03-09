package net.tecdroid.wrappers

import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DutyCycleEncoder
import net.tecdroid.util.NumericId
import net.tecdroid.util.units.rotations

class ThroughBoreAbsoluteEncoder(port: NumericId, private val inverted: Boolean) {
    private val encoder: DutyCycleEncoder = DutyCycleEncoder(port.id)
    private val reading : Angle = Rotations.of(encoder.get())

    val position: Angle
        get() = if (inverted) invertReading(reading) else reading

    private fun invertReading(angle: Angle) = (1.0.rotations - angle)
}