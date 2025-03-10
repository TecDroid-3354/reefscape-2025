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
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.constants.subsystemTabName
import net.tecdroid.subsystems.util.generic.IdentifiableSubsystem
import net.tecdroid.subsystems.util.generic.WithAbsoluteEncoders
import net.tecdroid.subsystems.util.identification.GenericSysIdRoutine
import net.tecdroid.util.units.clamp
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

class Wrist(internal val config: WristConfig) : IdentifiableSubsystem(), Sendable, WithAbsoluteEncoders {
    internal val motorController = TalonFX(config.motorControllerId.id)

    private val absoluteEncoder =
        ThroughBoreAbsoluteEncoder(
            port = config.absoluteEncoderPort,
            offset = config.absoluteEncoderOffset,
            inverted = config.absoluteEncoderIsInverted
        )

    init {
        configureMotorInterface()
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        motorController.setControl(request)
    }

    fun setAngle(newAngle: Angle) {
        val targetAngle = clamp(config.minimumAngle, config.maximumAngle, newAngle)
        val request = MotionMagicVoltage(config.gearRatio.unapply(targetAngle)).withSlot(0)
        motorController.setControl(request)
    }

    fun setAngleCommand(newAngle: Angle): Command = Commands.runOnce({ setAngle(newAngle) }, this)

    override val power: Double
        get() = motorController.get()

    override val motorPosition: Angle
        get() = motorController.position.value

    override val motorVelocity: AngularVelocity
        get() = motorController.velocity.value

    val angle: Angle
        get() =
            config.gearRatio.apply(motorPosition)

    private val absoluteAngle: Angle
        get() = absoluteEncoder.position

    override fun matchRelativeEncodersToAbsoluteEncoders() {
        motorController.setPosition(config.gearRatio.unapply(absoluteAngle))
    }

    // ///////////// //
    // Configuration //
    // ///////////// //

    private fun configureMotorInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.positiveDirection.toInvertedValue())

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
                .withMotionMagicCruiseVelocity(config.gearRatio.unapply(config.motionTargets.cruiseVelocity))
                .withMotionMagicAcceleration(config.gearRatio.unapply(config.motionTargets.acceleration))
                .withMotionMagicJerk(config.gearRatio.unapply(config.motionTargets.jerk))
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

    fun publishToShuffleboard() {
        val tab = Shuffleboard.getTab(subsystemTabName)
        tab.add("Wrist" , this)
    }

    @SuppressWarnings("unusued")
    fun createIdentificationRoutine() = GenericSysIdRoutine(
        name = "Wrist",
        subsystem = this,
        forwardsRunningCondition = { angle < config.maximumAngle },
        backwardsRunningCondition = { angle > config.minimumAngle }
    )
}
