package net.tecdroid.subsystems.climber

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
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.constants.subsystemTabName
import net.tecdroid.subsystems.util.generic.VoltageControlledSubsystem
import net.tecdroid.subsystems.util.generic.WithAbsoluteEncoders
import net.tecdroid.util.units.clamp
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

class Climber(private val config: ClimberConfig) : SubsystemBase(), Sendable, VoltageControlledSubsystem, WithAbsoluteEncoders
 {
    private val leadMotorController = TalonFX(config.leadingMotorId.id)
    private val followerMotorController = TalonFX(config.followerMotorId.id)
    private val absoluteEncoder = ThroughBoreAbsoluteEncoder(
        port = config.absoluteEncoderPort,
        offset = config.absoluteEncoderOffset,
        inverted = config.absoluteEncoderInverted
    )

    init {
        configureMotors()
    }

     override fun setVoltage(voltage: Voltage) {
         val request = VoltageOut(voltage)
         leadMotorController.setControl(request)
     }

     fun setAngle(newAngle: Angle) {
         val targetAngle = clamp(config.minimumAngle, config.maximumAngle, newAngle)
         val request = MotionMagicVoltage(config.gearRatio.unapply(targetAngle)).withSlot(0)
         leadMotorController.setControl(request)
     }

     fun setAngleCommand(newAngle: Angle): Command = Commands.runOnce({ setAngle(newAngle) })

     internal val motorPosition: Angle
         get() = leadMotorController.position.value

     internal val motorVelocity: AngularVelocity
         get() = leadMotorController.velocity.value

     val angle: Angle
         get() = config.gearRatio.apply(motorPosition)

     val absoluteAngle: Angle
         get() = absoluteEncoder.position

     override fun matchRelativeEncodersToAbsoluteEncoders() {
         leadMotorController.setPosition(config.gearRatio.unapply(absoluteAngle))
     }

    private fun configureMotors() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted((config.positiveRotationDirection).toInvertedValue())

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
                .withMotionMagicCruiseVelocity(config.gearRatio.unapply(config.motionTargets.cruiseVelocity))
                .withMotionMagicAcceleration(config.gearRatio.unapply(config.motionTargets.acceleration))
                .withMotionMagicJerk(config.gearRatio.unapply(config.motionTargets.jerk))
        }

        leadMotorController.clearStickyFaults()
        leadMotorController.configurator.apply(talonConfig)

        followerMotorController.clearStickyFaults()
        followerMotorController.configurator.apply(talonConfig)

        followerMotorController.setControl(Follower(leadMotorController.deviceID, false))
    }

     override fun initSendable(builder: SendableBuilder) {
         with(builder) {
             addDoubleProperty("Current Angle (Rotations)", { angle.`in`(Rotations) }) {}
             addDoubleProperty("Absolute Angle (Rotations)", { absoluteAngle.`in`(Rotations) }) {}
         }
     }

     fun publishToShuffleboard() {
         val tab = Shuffleboard.getTab(subsystemTabName)
         tab.add("Climber" , this)
     }
}
