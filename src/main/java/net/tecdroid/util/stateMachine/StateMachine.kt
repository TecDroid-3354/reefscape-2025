package net.tecdroid.util.stateMachine

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

enum class States(var config: StateConfig) {
    CoralState(StateConfig()),
    AlgaeState(StateConfig()),
    ScoreState(StateConfig()),
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

enum class Phase() {
    Teleop,
    Simulation,
    Autonomous,
    All
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

    /**
     * @return The current state
     */
    fun getCurrentState() : States = currentState

    /**
     * Verify if the current state its equal to the param one
     * @param state state to compare
     * @return If the param state its equal to the current one
     */
    fun isState(state: States): () -> Boolean = { currentState == state }

    /**
     * Change the command that its always executed when you change to another state
     * @param command Change command
     */
    fun setChangeCommand(command: Command) {
        changeStateCommand = command
    }

    // Condition system
    private val generalConditions = mutableListOf<Pair<() -> Boolean, States>>()
    private val teleopConditions = mutableListOf<Pair<() -> Boolean, States>>()
    private val simulationConditions = mutableListOf<Pair<() -> Boolean, States>>()
    private val autoConditions = mutableListOf<Pair<() -> Boolean, States>>()

    /**
     * Add a condition that if true, it change the state. This condition is
     * evaluated periodically depending on his phase execution
     * @param condition Condition that change the state
     * @param state Target State
     * @param executionPhase Phase in which you will evaluate the condition
     */

    fun addCondition(condition: () -> Boolean, state: States, executionPhase: Phase) {
        when(executionPhase) {
            Phase.Teleop -> teleopConditions.add(condition to state)
            Phase.Simulation -> simulationConditions.add(condition to state)
            Phase.Autonomous -> autoConditions.add(condition to state)
            Phase.All -> generalConditions.add(condition to state)
        }
    }

    private fun assess(conditions : MutableList<Pair<() -> Boolean, States>>) {
        // Check all the conditions
        for ((condition, state) in conditions) {
            // Check if we are not trying to change to the same state
            if (state != currentState && condition()) {
                changeState(state)
            }
        }
    }

    override fun periodic() {
        // Evaluate each condition depending on his execute phase
        if (DriverStation.isAutonomous()) {
            assess(autoConditions)
            assess(generalConditions)
        } else if (DriverStation.isTeleop()) {
            assess(teleopConditions)
            assess(generalConditions)
        } else if (DriverStation.isTest()) {
            assess(simulationConditions)
            assess(generalConditions)
        }
    }
}