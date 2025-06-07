package net.tecdroid.subsystems.elevator

import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.NeutralModeValue.Brake
import com.ctre.phoenix6.signals.NeutralModeValue.Coast
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.*
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.subsystems.util.generic.*
import net.tecdroid.subsystems.util.motors.KrakenMotors

class Elevator(private val config: ElevatorConfig) :
    TdSubsystem("Elevator"),
    MeasurableSubsystem,
    LinearSubsystem,
    LoggableSubsystem,
    VoltageControlledSubsystem
{
    private val motorsConfig = KrakenMotors.createTalonFullConfig( // Stores the shared configuration of the motors.
        KrakenMotors.configureMotorOutputs(Brake, config.motorDirection.toInvertedValue()),
        KrakenMotors.configureCurrentLimits(config.motorCurrentLimit, null),
        KrakenMotors.configureSlot0(config.controlGains), config.reduction,
        null, config.motionTargets, config.sprocket
    )
    private val leadMotorController = KrakenMotors.createTalonWithCustomConfig(
        config.leadMotorControllerId, motorsConfig
    )
    private val followerMotorController = KrakenMotors.createFollowerTalonWithCustomConfig(
        config.followerMotorId, config.leadMotorControllerId, true, motorsConfig
    )
    private var target: Angle

    override val forwardsRunningCondition = { displacement < config.measureLimits.relativeMaximum }
    override val backwardsRunningCondition = { displacement > config.measureLimits.relativeMinimum }

    init {
        publishToShuffleboard()
        target = motorPosition
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        leadMotorController.setControl(request)
    }

    override fun setDisplacement(targetDisplacement: Distance) {
        val clampedDisplacement = config.measureLimits.coerceIn(targetDisplacement) as Distance
        val targetAngle = config.sprocket.linearDisplacementToAngularDisplacement(clampedDisplacement)
        val transformedAngle = config.reduction.unapply(targetAngle)
        val request = MotionMagicVoltage(transformedAngle)

        target = transformedAngle
        leadMotorController.setControl(request)
    }

    fun getPositionError(): Angle =
        if (target > motorPosition) target - motorPosition else motorPosition - target

    override val power: Double
        get() = leadMotorController.get()

    override val motorPosition: Angle
        get() = leadMotorController.position.value

    override val motorVelocity: AngularVelocity
        get() = leadMotorController.velocity.value

    override val displacement: Distance
        get() = config.sprocket.angularDisplacementToLinearDisplacement(config.reduction.apply(motorPosition))

    val x = {displacement}

    override val velocity: LinearVelocity
        get() = config.sprocket.angularVelocityToLinearVelocity(config.reduction.apply(motorVelocity))

    fun coast(): Command = Commands.runOnce({
        leadMotorController.setNeutralMode(Coast)
        followerMotorController.setNeutralMode(Coast)
    })

    fun brake(): Command = Commands.runOnce({
        leadMotorController.setNeutralMode(Brake)
        followerMotorController.setNeutralMode(Brake)
    })

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current Displacement (Meters)", { displacement.`in`(Meters) }, {})
            addDoubleProperty("Inverse Operation (Rotations)", { motorPosition.`in`(Rotations) }, {})
        }
    }
}

