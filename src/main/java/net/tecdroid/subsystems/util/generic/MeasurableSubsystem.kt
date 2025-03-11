package net.tecdroid.subsystems.util.generic

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Commands

interface MeasurableSubsystem : VoltageControlledSubsystem {
    val motorPosition: Angle
    val motorVelocity: AngularVelocity
    val power: Double
}

interface AngularSubsystem {
    val angle: Angle
    val angularVelocity: AngularVelocity

    fun setAngle(targetAngle: Angle)
    fun setAngleCommand(targetAngle: Angle) = Commands.runOnce({ setAngle(targetAngle) })
}

interface LinearSubsystem {
    val displacement: Distance
    val velocity: LinearVelocity

    fun setDisplacement(targetDisplacement: Distance)
    fun setDisplacementCommand(targetDisplacement: Distance) = Commands.runOnce({ setDisplacement(targetDisplacement) })
}
