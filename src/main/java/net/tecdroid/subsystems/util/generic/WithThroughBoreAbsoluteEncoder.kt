package net.tecdroid.subsystems.util.generic

import edu.wpi.first.units.measure.Angle
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

interface WithThroughBoreAbsoluteEncoder {
    val absoluteEncoder: ThroughBoreAbsoluteEncoder
    val absoluteAngle: Angle
        get() = absoluteEncoder.position

    fun onMatchRelativeEncodersToAbsoluteEncoders()

    fun matchRelativeEncodersToAbsoluteEncoders() {
        require(this is TdSubsystem) { "Classes With (a) Through Bore Absolute Encoder must be Subsystems" }
        onMatchRelativeEncodersToAbsoluteEncoders()
    }
}