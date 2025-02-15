package net.tecdroid.input

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import net.tecdroid.util.*

object WpiLibControllerReferenceFrame: ReferenceFrame2d(Axis2d.Y, LongitudinalDirection.Back, Axis2d.X, TransversalDirection.Right)

/**
 * Overrides the default behavior of existing WPILib controllers in order to follow the team's standard conventions
 */
class Controller(id: NumericId) : CommandXboxController(id.id) {
    override fun getLeftX(): Double {
        return ConventionEnforcer.negateToConform(super.getLeftX())
    }

    override fun getLeftY(): Double {
        return ConventionEnforcer.negateToConform(super.getLeftY())
    }

    override fun getRightX(): Double {
        return ConventionEnforcer.negateToConform(super.getRightX())
    }

    override fun getRightY(): Double {
        return ConventionEnforcer.negateToConform(super.getRightY())
    }

}