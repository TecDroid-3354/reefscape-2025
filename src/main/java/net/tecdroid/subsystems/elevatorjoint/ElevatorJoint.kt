package net.tecdroid.subsystems.elevatorjoint

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import net.tecdroid.subsystems.util.generic.*
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

class ElevatorJoint(private val config: ElevatorJointConfig) :
    TdSubsystem("Elevator Joint"),
    MeasurableSubsystem,
    AngularSubsystem,
    LoggableSubsystem,
    VoltageControlledSubsystem,
    WithThroughBoreAbsoluteEncoder {
    private val leadMotorController = TalonFX(config.leadMotorControllerId.id)
    private val followerMotorController = TalonFX(config.followerMotorId.id)

    override val absoluteEncoder =
        ThroughBoreAbsoluteEncoder(
            port = config.absoluteEncoderPort,
            offset = config.absoluteEncoderOffset,
            inverted = config.absoluteEncoderIsInverted
        )

    override val forwardsRunningCondition  = { angle < config.limits.relativeMaximum }
    override val backwardsRunningCondition = { angle > config.limits.relativeMinimum }

    init {
        configureMotorsInterface()
        matchRelativeEncodersToAbsoluteEncoders()
        publishToShuffleboard()
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        leadMotorController.setControl(request)
    }

    override fun setAngle(targetAngle: Angle) {
        val clampedAngle = config.limits.coerceIn(targetAngle) as Angle
        val transformedAngle = config.reduction.unapply(clampedAngle)
        SmartDashboard.putNumber("TYA", clampedAngle.`in`(Rotations))
        val request = MotionMagicVoltage(transformedAngle)
        leadMotorController.setControl(request)
    }

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

    private fun configureMotorsInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(config.positiveDirection.toInvertedValue())

            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.currentLimit)

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


        leadMotorController.clearStickyFaults()
        followerMotorController.clearStickyFaults()

        leadMotorController.configurator.apply(talonConfig)
        followerMotorController.configurator.apply(talonConfig)

        followerMotorController.setControl(Follower(leadMotorController.deviceID, true))
    }

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current Angle (Rotations)", { angle.`in`(Rotations) }, {})
            addDoubleProperty("Current Absolute Angle (Rotations)", { absoluteAngle.`in`(Rotations) }, {})
        }
    }
}
