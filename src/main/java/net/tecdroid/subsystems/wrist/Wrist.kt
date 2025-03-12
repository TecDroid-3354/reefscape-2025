package net.tecdroid.subsystems.wrist

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.SendableBuilder
import net.tecdroid.subsystems.util.generic.*
import net.tecdroid.util.units.abs
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder
import kotlin.math.absoluteValue

class Wrist(private val config: WristConfig) :
    TdSubsystem("Wrist"),
    LoggableSubsystem,
    WithThroughBoreAbsoluteEncoder,
    AngularSubsystem {
    private val motorController = TalonFX(config.motorControllerId.id)
    private var target : Angle

    override val absoluteEncoder = ThroughBoreAbsoluteEncoder(
        port = config.absoluteEncoderPort,
        offset = config.absoluteEncoderOffset,
        inverted = config.absoluteEncoderIsInverted
    )

    override val forwardsRunningCondition  = { angle < config.limits.relativeMaximum }
    override val backwardsRunningCondition = { angle > config.limits.relativeMinimum }

    init {
        configureMotorInterface()
        matchRelativeEncodersToAbsoluteEncoders()
        publishToShuffleboard()
        target = motorPosition
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        motorController.setControl(request)
    }

    override fun setAngle(targetAngle: Angle) {
        val clampedAngle = config.limits.coerceIn(targetAngle) as Angle
        val transformedAngle = config.reduction.unapply(clampedAngle)
        val request = MotionMagicVoltage(transformedAngle).withSlot(0)

        target = transformedAngle
        motorController.setControl(request)
    }

    fun getPositionError(): Angle =
        if (target > motorPosition) target - motorPosition else motorPosition - target

    override val power: Double
        get() = motorController.get()

    override val motorPosition: Angle
        get() = motorController.position.value

    override val motorVelocity: AngularVelocity
        get() = motorController.velocity.value

    override val angle: Angle
        get() = config.reduction.apply(motorPosition)

    override val angularVelocity: AngularVelocity
        get() = config.reduction.apply(motorVelocity)

    override fun onMatchRelativeEncodersToAbsoluteEncoders() {
        motorController.setPosition(config.reduction.unapply(absoluteAngle))
    }

    private fun configureMotorInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.motorDirection.toInvertedValue())

            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.motorCurrentLimit)

            Slot0
                .withKP(config.controlGains.p)
                .withKI(config.controlGains.i)
                .withKD(config.controlGains.d)
                .withKS(config.controlGains.s)
                .withKV(config.controlGains.v)
                .withKA(config.controlGains.a)
                .withKG(config.controlGains.g)

            MotionMagic
                .withMotionMagicCruiseVelocity(config.reduction.unapply(config.motionTargets.cruiseVelocity))
                .withMotionMagicAcceleration(config.reduction.unapply(config.motionTargets.acceleration))
                .withMotionMagicJerk(config.reduction.unapply(config.motionTargets.jerk))
        }


        motorController.clearStickyFaults()
        motorController.configurator.apply(talonConfig)
    }

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current Angle (Rotations)", { angle.`in`(Rotations) }, {})
            addDoubleProperty("Current Absolute Angle (Rotations)", { absoluteAngle.`in`(Rotations) }, {})
        }
    }
}