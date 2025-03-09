package net.tecdroid.subsystems.wrist

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.subsystems.generic.VoltageControlledSubsystem
import net.tecdroid.subsystems.generic.WithAbsoluteEncoders
import net.tecdroid.util.units.clamp
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

class Wrist(private val config: WristConfig) : SubsystemBase(), VoltageControlledSubsystem, WithAbsoluteEncoders {
    private val motorController = TalonFX(config.motorControllerId.id)
    private val absoluteEncoder =
        ThroughBoreAbsoluteEncoder(config.absoluteEncoderPort, config.absoluteEncoderIsInverted)

    init {
        configureMotorInterface()
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        motorController.setControl(request)
    }

    fun setWristAngle(angle: Angle) {
        val targetAngle = clamp(angle, config.minimumAngle, config.maximumAngle) as Angle
        val request = MotionMagicVoltage(config.gearRatio.unapply(targetAngle)).withSlot(0)
        motorController.setControl(request)
    }

    val angle: Angle
        get() =
            config.gearRatio.apply(motorController.position.value)

    private val absoluteAngle: Angle
        get() = config.gearRatio.apply(absoluteEncoder.position)

    override fun matchRelativeEncodersToAbsoluteEncoders() {
        motorController.setPosition(absoluteAngle)
    }

    fun publishToShuffleboard() {
        val tab = Shuffleboard.getTab("Wrist")
        tab.addDouble("Current Angle (deg)") { angle.`in`(Units.Degrees) }
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
}



