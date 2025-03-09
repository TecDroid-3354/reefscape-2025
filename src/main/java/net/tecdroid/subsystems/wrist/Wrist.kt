package net.tecdroid.subsystems.wrist

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import net.tecdroid.subsystems.util.generic.VoltageControlledSubsystem
import net.tecdroid.subsystems.util.generic.WithAbsoluteEncoders
import net.tecdroid.subsystems.util.identification.AngularSysIdRoutine
import net.tecdroid.util.units.clamp
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

class Wrist(private val config: WristConfig) : SubsystemBase(), Sendable, VoltageControlledSubsystem, WithAbsoluteEncoders {
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

    fun setAngle(angle: Angle) {
        val targetAngle = clamp(angle, config.minimumAngle, config.maximumAngle) as Angle
        val request = MotionMagicVoltage(config.gearRatio.unapply(targetAngle)).withSlot(0)
        motorController.setControl(request)
    }

    fun setAngleCommand(angle: Angle): Command = Commands.runOnce({ setAngle(angle) }, this)

    internal val motorPosition: Angle
        get() = motorController.position.value

    internal val motorVelocity: AngularVelocity
        get() = motorController.velocity.value

    val angle: Angle
        get() =
            config.gearRatio.apply(motorPosition)

    val angularVelocity: AngularVelocity
        get() =
            config.gearRatio.apply(motorVelocity)

    private val absoluteAngle: Angle
        get() = config.gearRatio.apply(absoluteEncoder.position)

    override fun matchRelativeEncodersToAbsoluteEncoders() {
        motorController.setPosition(absoluteAngle)
    }

    fun publishToShuffleboard() {
        val tab = Shuffleboard.getTab("Wrist")
        tab.addDouble("Current Angle (deg)") { angle.`in`(Degrees) }
    }

    // ///////////// //
    // Configuration //
    // ///////////// //

    private fun configureMotorInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.gearRatio.transformRotation(config.positiveDirection).toInvertedValue())

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
            addDoubleProperty("Current Angle (Degrees)", { angle.`in`(Degrees) }, {})
            addDoubleProperty("Current Absolute Angle (Degrees)", { absoluteAngle.`in`(Degrees) }, {})
        }
    }
}

class WristSystemIdentificationRoutine(wrist: Wrist) : AngularSysIdRoutine() {
    override val routine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            wrist::setVoltage,
            { log: SysIdRoutineLog ->
                log.motor("Wrist Motor")
                   .voltage(voltage.mut_replace(RobotController.getBatteryVoltage() * wrist.motorController.get(), Volts))
                   .angularPosition(position.mut_replace(wrist.motorPosition))
                   .angularVelocity(velocity.mut_replace(wrist.motorVelocity))
            },
            wrist
        )
    )
}

