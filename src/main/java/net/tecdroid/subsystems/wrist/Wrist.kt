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
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.subsystems.util.generic.*
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder
import org.littletonrobotics.junction.Logger

class Wrist(private val io: WristIO, private val config: WristConfig) :
    TdSubsystem("Wrist"),
    LoggableSubsystem,
    WithThroughBoreAbsoluteEncoder,
    AngularSubsystem
{
    override val absoluteEncoder = io.getAbsoluteEncoderInstance()

    override val forwardsRunningCondition  = { angle < config.measureLimits.relativeMaximum }
    override val backwardsRunningCondition = { angle > config.measureLimits.relativeMinimum }

    override val power: Double
        get() = io.getMotorPower()

    override val motorPosition: Angle
        get() = io.getMotorPosition()

    override val motorVelocity: AngularVelocity
        get() = io.getMotorVelocity()

    override val angle: Angle
        get() = config.reduction.apply(motorPosition)

    override val angularVelocity: AngularVelocity
        get() = config.reduction.apply(motorVelocity)

    private var inputs = WristIOInputsAutoLogged() // Used to log all the subsystem's inputs.

    init {
        matchRelativeEncodersToAbsoluteEncoders()
        publishToShuffleboard()
        io.setTargetAngle(motorPosition)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Wrist", inputs)
    }

    override fun setVoltage(voltage: Voltage) {
        io.setVoltage(voltage)
    }

    override fun setAngle(targetAngle: Angle) {
        val clampedAngle = config.measureLimits.coerceIn(targetAngle) as Angle
        val transformedAngle = config.reduction.unapply(clampedAngle)
        io.setTargetAngle(transformedAngle)
        io.setAngle(transformedAngle)
    }

    override fun onMatchRelativeEncodersToAbsoluteEncoders() {
        io.setMotorPosition(config.reduction.unapply(absoluteAngle))
    }

    fun getPositionError(): Angle =
        if (io.getTargetAngle() > motorPosition) io.getTargetAngle() - motorPosition else motorPosition - io.getTargetAngle()

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current Angle (Rotations)", { angle.`in`(Rotations) }, {})
            addDoubleProperty("Current Absolute Angle (Rotations)", { absoluteAngle.`in`(Rotations) }, {})
        }
    }

    fun coast(): Command = Commands.runOnce({ io.coast() })
    fun brake(): Command = Commands.runOnce({ io.brake() })
}
