package net.tecdroid.util.stateMachine

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

/**
 * The states
 * @param initialCommand Command executed when this state inits
 * @param defaultCommand Command executed periodically
 * @param endCommand Command executed when we end the state
 */
data class StateConfig(
    val initialCommand : Command = Commands.none(),
    val defaultCommand : Command = Commands.none(),
    val endCommand : Command = Commands.none()
)