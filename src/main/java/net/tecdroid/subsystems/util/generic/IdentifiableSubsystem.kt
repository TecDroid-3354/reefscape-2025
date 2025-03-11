package net.tecdroid.subsystems.util.generic

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase

abstract class IdentifiableSubsystem : SubsystemBase(), VoltageControlledSubsystem {
    abstract val motorPosition: Angle
    abstract val motorVelocity: AngularVelocity
    abstract val power: Double
}
