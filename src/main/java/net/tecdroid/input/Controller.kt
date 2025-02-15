package net.tecdroid.input

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import net.tecdroid.util.*

/**
 * Overrides the default behavior of existing WPILib controllers in order to follow the team's standard conventions
 */
class Controller(id: NumericId) : CommandXboxController(id.id) {
    override fun getLeftX(): Double {
        return -(super.getLeftX())
    }

    override fun getLeftY(): Double {
        return -(super.getLeftY())
    }

    override fun getRightX(): Double {
        return -(super.getRightX())
    }

    override fun getRightY(): Double {
        return -(super.getRightY())
    }

}