package net.tecdroid.subsystems.elevatorjoint

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.subsystems.generic.VoltageControlledSubsystem
import net.tecdroid.subsystems.generic.WithAbsoluteEncoders
import net.tecdroid.util.units.clamp
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

class ElevatorJoint(private val config: ElevatorJointConfig) : SubsystemBase(), Sendable, VoltageControlledSubsystem,
    WithAbsoluteEncoders {
    private val leadMotorController = TalonFX(config.leadMotorControllerId.id)
    private val followerMotorController = TalonFX(config.followerMotorId.id)

    private val absoluteEncoder =
        ThroughBoreAbsoluteEncoder(
            port = config.absoluteEncoderPort,
            offset = config.absoluteEncoderOffset,
            inverted = config.absoluteEncoderIsInverted
        )

    init {
        configureMotorsInterface()
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        leadMotorController.setControl(request)
    }

    fun setTargetAngle(angle: Angle) {
        val targetAngle = clamp(config.minimumAngle, config.maximumAngle, angle) as Angle
        val request = MotionMagicVoltage(config.gearRatio.unapply(targetAngle))
        leadMotorController.setControl(request)
    }

    fun setTargetAngleCommand(angle: Angle): Command = Commands.runOnce({ setTargetAngle(angle) })

    val angle: Angle
        get() = config.gearRatio.apply(leadMotorController.position.value)

    private val absoluteAngle: Angle
        get() = config.gearRatio.apply(absoluteEncoder.position)

    override fun matchRelativeEncodersToAbsoluteEncoders() {
        leadMotorController.setPosition(config.gearRatio.unapply(absoluteAngle))
    }

    fun publishToShuffleboard() {
        val tab = Shuffleboard.getTab("Elevator Joint")
        tab.addString("Current Angle") { angle.toString() }
        tab.addString("Current Absolute Angle") { absoluteAngle.toString() }
    }

    private fun configureMotorsInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.gearRatio.transformRotation(config.positiveDirection).toInvertedValue())

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
        followerMotorController.clearStickyFaults()

        leadMotorController.configurator.apply(talonConfig)
        followerMotorController.configurator.apply(talonConfig)

        followerMotorController.setControl(Follower(leadMotorController.deviceID, true))
    }

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current Angle (Degrees)", { angle.`in`(Degrees) }, {})
            addDoubleProperty("Current Absolute Angle (Degrees)", { absoluteAngle.`in`(Degrees) }, {})
        }
    }
}


