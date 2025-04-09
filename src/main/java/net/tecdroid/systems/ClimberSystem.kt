package net.tecdroid.systems

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import net.tecdroid.input.CompliantXboxController

import net.tecdroid.subsystems.climber.climberElevator.ClimberElevator
import net.tecdroid.subsystems.climber.climberElevator.ClimberElevatorConfig
import net.tecdroid.subsystems.climber.climberIntake.ClimberIntake
import net.tecdroid.subsystems.climber.climberIntake.ClimberIntakeConfig

import net.tecdroid.util.meters
import net.tecdroid.util.volts


data class ClimberPose(
    var climberElevatorDisplacement: Distance,
    var targetVoltage: Voltage
)

enum class ClimberPoses(var pose: ClimberPose) {
    Bottom(ClimberPose(
        climberElevatorDisplacement = 0.0.meters, // TODO: CHECK THIS NUMBER
        targetVoltage = 0.0.volts
    )),
    Top(ClimberPose(
        climberElevatorDisplacement = 0.0.meters, // TODO: CHECK THIS NUMBER
        targetVoltage = 0.0.volts
    ))
}

enum class ClimberTarget {
    Top, Bottom
}

class ClimberSystem(climberIntakeConfig: ClimberIntakeConfig, climberElevatorConfig: ClimberElevatorConfig) : Sendable {
    private val climberIntake = ClimberIntake(climberIntakeConfig)
    private val climberElevator = ClimberElevator(climberElevatorConfig)
    private var targetVoltage = 0.0.volts
    private val elevatorDisplacementVoltage = 10.0.volts

    private fun setClimberElevatorDisplacement(displacement: Distance) : Command = climberElevator.setDisplacementCommand(displacement)
    private fun enableClimberIntake() : Command = climberIntake.setVoltageCommand({ 12.0.volts }) // TODO: CHECK VOLT NUMBER
    private fun disableClimberIntake() : Command = climberIntake.setVoltageCommand({ 0.0.volts })

    private fun setPoseCommand(pose: ClimberPose) : Command {
        return SequentialCommandGroup(
            setClimberElevatorDisplacement(pose.climberElevatorDisplacement)
                .until( {climberElevator.isTopLimitSwitchPressed() || climberElevator.isBottomLimitSwitchPressed()} ),
            Commands.runOnce({ targetVoltage = pose.targetVoltage })
        )
    }

    fun sendTo(target: ClimberTarget): Command = when (target) {
        ClimberTarget.Top ->
            climberElevator.setVoltageCommand { elevatorDisplacementVoltage }.until { climberElevator.isTopLimitSwitchPressed() }
        ClimberTarget.Bottom ->
            climberElevator.setVoltageCommand { -elevatorDisplacementVoltage }.until { climberElevator.isBottomLimitSwitchPressed() }
    }

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Climber Elevator Displacement (Meters)", { climberElevator.displacement.`in`(Meters) }) {}
        }
    }

    fun assignCommandsToController(controller: CompliantXboxController) {
        controller.povUp().onTrue(
            Commands.sequence(
                setPoseCommand(ClimberPoses.Top.pose),
                enableClimberIntake() // use just in case the intake function aint working properly
            )
        )
    }

    fun coastMode() : Command {
        return Commands.sequence(
            climberIntake.coast(),
            climberElevator.coast()
        )
    }

    fun brakeMode() : Command {
        return Commands.sequence(
            climberIntake.brake(),
            climberElevator.brake()
        )
    }
}