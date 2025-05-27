package net.tecdroid.util

enum class States{
    IntakeState, ScoreState, AlgaeState
}

class State(private var state: States) {
    fun getState(): States = state

    fun changeState(state: States) {
        this.state =  state
    }

    fun isState(state: States) = { this.state == state }

    fun toggleCoralModeAndAlgaeMode() {
        if (isState(States.IntakeState).invoke()) {
            state = States.AlgaeState
        } else if (isState(States.AlgaeState).invoke()) {
            state = States.IntakeState
        }
    }
}