package net.tecdroid.subsystems.util.generic

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import net.tecdroid.constants.subsystemTabName

interface LoggableSubsystem: Sendable {
    fun publishToShuffleboard() {
        require(this is TdSubsystem) { "Loggable Subsystems must be Subsystems" }

        val tab = Shuffleboard.getTab(subsystemTabName)
        tab.add(this.name, this)
    }
}