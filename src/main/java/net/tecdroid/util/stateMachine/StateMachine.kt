package net.tecdroid.util.stateMachine

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.BooleanSupplier

enum class States(var config: StateConfig) {
    ScoreState(StateConfig()),
    AlgaeState(StateConfig()),
    IntakeState(StateConfig());

    /**
     * Change the states commands. These classes are global, so the config
     * will change for all the instances
     * @param config The class of commands
     */
    fun setConfig(config: StateConfig) {
        this.config = config
    }
}

/**
 * The states
 * @param currentState Initial State
 * @param changeStateCommand Command executed when we change the state
 */
class StateMachine(private var currentState: States,
                   private var changeStateCommand: Command = Commands.none()) : SubsystemBase() {
    init {
        this.defaultCommand = currentState.config.defaultCommand
        currentState.config.initialCommand.execute()
    }

    // Change the state default command
    private fun changeStateDefaultCommand(defaultCommand: Command) {
        this.defaultCommand = defaultCommand
    }

    /**
     * Change the state, set the new default command, execute the end command of the last state and
     * execute the change commands (General command executed when we change to another state).
     * @param state New state
     */

    fun changeState(state: States) {
        // Verify if our state isn't the same
        if (state !=  currentState) {
            Commands.sequence(
                // execute end command
                currentState.config.endCommand,
                // execute the general command
                changeStateCommand,
                // execute the initial command of the new state
                state.config.initialCommand
            ).execute()

            // set the new default command
            changeStateDefaultCommand(state.config.defaultCommand)

            // Change the state
            currentState = state
        }
    }

    fun getCurrentState() : States = currentState

    fun isState(state: States): () -> Boolean = { currentState == state }

    fun setChangeCommand(command: Command) {
        changeStateCommand = command
    }

    fun toggleCoralModeAndAlgaeMode() {
        if (isState(States.IntakeState).invoke()) {
            changeState(States.AlgaeState)
        } else if (isState(States.AlgaeState).invoke()) {
            changeState(States.IntakeState)
        }
    }

    // Condition system
    private val conditions = mutableListOf<Pair<() -> Boolean, States>>()

    fun addCondition(condition: () -> Boolean, state: States) {
        conditions.add(condition to state)
    }

    fun assess() {
        for ((condition, state) in conditions) {
            if (condition()) {
                changeState(state)
            }
        }
    }

    override fun periodic() {
        assess()
    }
}