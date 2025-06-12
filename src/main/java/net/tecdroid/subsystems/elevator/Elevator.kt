package net.tecdroid.subsystems.elevator

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.*
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.subsystems.util.generic.*
import org.littletonrobotics.junction.Logger

class Elevator(private val io: ElevatorIO, private val config: ElevatorConfig) :
    TdSubsystem("Elevator"),
    MeasurableSubsystem,
    LinearSubsystem,
    LoggableSubsystem,
    VoltageControlledSubsystem
{
    private val inputs = ElevatorIO.ElevatorIOInputs()

    override val forwardsRunningCondition = { displacement < config.measureLimits.relativeMaximum }
    override val backwardsRunningCondition = { displacement > config.measureLimits.relativeMinimum }

    init {
        publishToShuffleboard()
        io.setTargetAngle(io.getMotorPosition())
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Elevator", inputs)
    }

    override fun setVoltage(voltage: Voltage) {
        io.setVoltage(voltage)
    }

    override fun setDisplacement(targetDisplacement: Distance) {
        val clampedDisplacement = config.measureLimits.coerceIn(targetDisplacement) as Distance
        val targetAngle = config.sprocket.linearDisplacementToAngularDisplacement(clampedDisplacement)
        val transformedAngle = config.reduction.unapply(targetAngle)

        io.setTargetAngle(transformedAngle)
        io.setAngularDisplacement(transformedAngle)
    }

    fun getPositionError(): Angle =
        if (io.getTargetAngle() > motorPosition) io.getTargetAngle() - motorPosition else motorPosition - io.getTargetAngle()

    override val power: Double
        get() = io.getMotorPower()

    override val motorPosition: Angle
        get() = io.getMotorPosition()

    override val motorVelocity: AngularVelocity
        get() = io.getMotorVelocity()

    override val displacement: Distance
        get() = config.sprocket.angularDisplacementToLinearDisplacement(config.reduction.apply(motorPosition))

    override val velocity: LinearVelocity
        get() = config.sprocket.angularVelocityToLinearVelocity(config.reduction.apply(motorVelocity))

    fun coast(): Command = Commands.runOnce({ io.coast() })

    fun brake(): Command = Commands.runOnce({ io.brake() })

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current Displacement (Meters)", { displacement.`in`(Meters) }, {})
            addDoubleProperty("Inverse Operation (Rotations)", { motorPosition.`in`(Rotations) }, {})
        }
    }
}

