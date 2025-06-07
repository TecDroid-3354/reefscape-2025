package net.tecdroid.subsystems.elevatorjoint

import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.NeutralModeValue.Brake
import com.ctre.phoenix6.signals.NeutralModeValue.Coast
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.subsystems.util.generic.*
import net.tecdroid.subsystems.util.motors.KrakenMotors
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

class ElevatorJoint(private val config: ElevatorJointConfig) :
    TdSubsystem("Elevator Joint"),
    MeasurableSubsystem,
    AngularSubsystem,
    LoggableSubsystem,
    VoltageControlledSubsystem,
    WithThroughBoreAbsoluteEncoder {
    private val motorsConfig = KrakenMotors.createTalonFullConfig( // Stores the shared configuration of the motors.
        KrakenMotors.configureMotorOutputs(Brake, config.motorDirection.toInvertedValue()),
        KrakenMotors.configureCurrentLimits(config.motorCurrentLimit, null),
        KrakenMotors.configureSlot0(config.controlGains), config.reduction,
        config.motionTargets, null, null
    )
    private val leadMotorController = KrakenMotors.createTalonWithCustomConfig(
        config.leadMotorControllerId, motorsConfig
    )
    private val followerMotorController = KrakenMotors.createFollowerTalonWithCustomConfig(
        config.followerMotorControllerId, config.leadMotorControllerId, false, motorsConfig
    )
    private var target: Angle

    override val absoluteEncoder =
        ThroughBoreAbsoluteEncoder(
            port = config.absoluteEncoderPort,
            offset = config.absoluteEncoderOffset,
            inverted = config.absoluteEncoderIsInverted
        )

    override val forwardsRunningCondition  = { angle < config.measureLimits.relativeMaximum }
    override val backwardsRunningCondition = { angle > config.measureLimits.relativeMinimum }

    init {
        matchRelativeEncodersToAbsoluteEncoders()
        publishToShuffleboard()
        target = motorPosition
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        leadMotorController.setControl(request)
    }

    override fun setAngle(targetAngle: Angle) {
        val clampedAngle = config.measureLimits.coerceIn(targetAngle) as Angle
        val transformedAngle = config.reduction.unapply(clampedAngle)
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

    override val angle: Angle
        get() = config.reduction.apply(motorPosition)

    override val angularVelocity: AngularVelocity
        get() = config.reduction.apply(motorVelocity)

    override fun onMatchRelativeEncodersToAbsoluteEncoders() {
        leadMotorController.setPosition(config.reduction.unapply(absoluteAngle))
    }

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current Angle (Rotations)", { angle.`in`(Rotations) }, {})
            addDoubleProperty("Current Absolute Angle (Rotations)", { absoluteAngle.`in`(Rotations) }, {})
        }
    }

    fun coast(): Command = Commands.runOnce({
        leadMotorController.setNeutralMode(Coast)
        followerMotorController.setNeutralMode(Coast)
    })

    fun brake(): Command = Commands.runOnce({
        leadMotorController.setNeutralMode(Brake)
        followerMotorController.setNeutralMode(Brake)
    })
}
