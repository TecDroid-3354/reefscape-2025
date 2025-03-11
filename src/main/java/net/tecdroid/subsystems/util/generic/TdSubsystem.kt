package net.tecdroid.subsystems.util.generic

import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.subsystems.util.identification.GenericSysIdRoutine

abstract class TdSubsystem(name: String): SubsystemBase(name), MeasurableSubsystem, VoltageControlledSubsystem {
    abstract val forwardsRunningCondition: () -> Boolean
    abstract val backwardsRunningCondition: () -> Boolean

    fun createIdentificationRoutine() : GenericSysIdRoutine {
        return GenericSysIdRoutine(
            name = name,
            subsystem = this,
            forwardsRunningCondition = forwardsRunningCondition,
            backwardsRunningCondition = backwardsRunningCondition
        )
    }
}