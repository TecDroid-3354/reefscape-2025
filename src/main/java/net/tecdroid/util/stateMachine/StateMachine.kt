package net.tecdroid.util.stateMachine

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.jgrapht.Graph
import org.jgrapht.graph.DefaultDirectedGraph
import org.jgrapht.graph.DefaultEdge


enum class States(var config: StateConfig) {
    CoralState(StateConfig()),
    AlgaeState(StateConfig()),
    ScoreState(StateConfig()),
    IntakeState(StateConfig());

    /**
     * Change the initial state command. These classes are global, so the config
     * will change for all the instances
     * @param command the command that will execute at the beggining of the state
     */

    fun setInitialCommand(command: Command) {
        this.config.initialCommand = command
    }

    /**
     * Change the default state command. These classes are global, so the config
     * will change for all the instances
     * @param command the command that will execute periodically during of the state
     */

    fun setDefaultCommand(command: Command) {
        this.config.defaultCommand = command
    }

    /**
     * Change the end state command. These classes are global, so the config
     * will change for all the instances
     * @param command the command that will execute at the end of the state
     */

    fun setEndCommand(command: Command) {
        this.config.endCommand = command
    }
}

enum class Phase {
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
class StateMachine(private var currentState: States) : SubsystemBase() {
    init {
        val initialDefaultCommand : Command = currentState.config.defaultCommand
        initialDefaultCommand.addRequirements(this)

        defaultCommand = initialDefaultCommand

        currentState.config.initialCommand.schedule()
    }

    class EdgeCommand : DefaultEdge() {
        var command: Command = Commands.none()
        var restricted : Boolean = false
    }

    private val graph: Graph<States, EdgeCommand> = DefaultDirectedGraph<States, EdgeCommand>(EdgeCommand::class.java)


    // Change the state default command
    private fun changeStateDefaultCommand(defaultCommand: Command) {
        removeDefaultCommand()
        defaultCommand.addRequirements(this)
        setDefaultCommand(defaultCommand)
    }

    /**
     * Change the state, set the new default command, execute the end command of the last state and
     * execute the change commands (General command executed when we change to another state).
     * @param targetState New state
     */

    fun changeState(targetState: States) {
        // Verify if our state isn't the same
        if (targetState !=  currentState) {
            transition(targetState)
        }
    }

    private fun transition(targetState: States) {
        val edge = graph.getEdge(currentState, targetState)

        // Check if we have a complex relation
        if (edge != null && !edge.restricted) {
            edge.command.schedule()

            // Normal transition

            // execute end command
            currentState.config.endCommand.schedule()

            // execute the initial command of the new state
            targetState.config.initialCommand.schedule()

            // set the new default command
            changeStateDefaultCommand(targetState.config.defaultCommand)

            // Change the state
            currentState = targetState
        } else {
            // Normal transition

            // execute end command
            currentState.config.endCommand.schedule()

            // execute the initial command of the new state
            targetState.config.initialCommand.schedule()

            // set the new default command
            changeStateDefaultCommand(targetState.config.defaultCommand)

            // Change the state
            currentState = targetState
        }

    }

    fun addEdge(from: States, to: States, edgeCommand: Command, restricted : Boolean) {
        val edgeCmd = EdgeCommand()
        edgeCmd.command = edgeCommand
        edgeCmd.restricted = restricted

        graph.addEdge(from, to, edgeCmd)
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
        } else if (DriverStation.isTeleop()) {
            assess(teleopConditions)
        } else if (DriverStation.isTest()) {
            assess(simulationConditions)
        }

        assess(generalConditions)
    }
}