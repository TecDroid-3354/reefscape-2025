package net.tecdroid.input

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import net.tecdroid.util.NumericId

/**
 * Overrides the default behavior of existing WPILib controllers in order to follow the team's standard conventions
 */
class CompliantXboxController(id: NumericId) : CommandXboxController(id.id) {
    override fun getLeftX(): Double = -super.getLeftX()
    override fun getLeftY(): Double = -super.getLeftY()

    override fun getRightX(): Double = -super.getRightX()
    override fun getRightY(): Double = -super.getRightY()
}