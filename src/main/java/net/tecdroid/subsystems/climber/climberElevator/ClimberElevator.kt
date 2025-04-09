package net.tecdroid.subsystems.climber.climberElevator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.subsystems.climber.climberIntake.ClimberIntakeConfig
import net.tecdroid.subsystems.util.generic.*

class ClimberElevator(private val config: ClimberElevatorConfig) :
    TdSubsystem("ElevatorClimber"),
    MeasurableSubsystem,
    LinearSubsystem,
    LoggableSubsystem,
    VoltageControlledSubsystem
{
    private val motorController = TalonFX(config.motorControllerId.id)
    private var target: Angle

    // Reference ids from config
    private val topLimitSwitch = DigitalInput(config.topLimitSwitchId.id); // TODO check channel
    private val bottomLimitSwitch = DigitalInput(config.bottomLimitSwitchId.id); // TODO check channel

    fun isTopLimitSwitchPressed() = topLimitSwitch.get()
    fun isBottomLimitSwitchPressed() = bottomLimitSwitch.get()

    override val forwardsRunningCondition = { isTopLimitSwitchPressed() }
    override val backwardsRunningCondition = { isBottomLimitSwitchPressed() }

    init {
        configureMotors()
        target = motorPosition
    }

    override val motorPosition: Angle
        get() = motorController.position.value

    override val motorVelocity: AngularVelocity
        get() = motorController.velocity.value

    override val power: Double
        get() = motorController.get()

    override val displacement: Distance
        get() = config.shaftDiameter.angularDisplacementToLinearDisplacement(config.reduction.apply(motorPosition))

    override val velocity: LinearVelocity
        get() = config.shaftDiameter.angularVelocityToLinearVelocity(config.reduction.apply(motorVelocity))

    val x = { displacement }

    override fun setVoltage(voltage: Voltage) {
        val output = VoltageOut(voltage)
        motorController.setControl(output)
    }

    override fun setDisplacement(targetDisplacement: Distance) {
        // This is ok. Didn't know you were using MeasureLimits
        val clampedDisplacement = config.measureLimits.coerceIn(targetDisplacement) as Distance
        val targetAngle = config.shaftDiameter.linearDisplacementToAngularDisplacement(clampedDisplacement)
        val transformedAngle = config.reduction.unapply(targetAngle)
        val request = MotionMagicVoltage(transformedAngle)

        target = transformedAngle
        motorController.setControl(request)
    }

    private fun configureMotors() {
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
                .withKI(config.controlGains.d)
                .withKS(config.controlGains.s)
                .withKV(config.controlGains.v)
                .withKA(config.controlGains.a)
                .withKG(config.controlGains.g)

            MotionMagic
                .withMotionMagicCruiseVelocity(config.reduction.unapply(config.motionTargets.angularVelocity(config.shaftDiameter)))
                .withMotionMagicAcceleration(config.reduction.unapply(config.motionTargets.angularAcceleration(config.shaftDiameter)))
                .withMotionMagicJerk(config.reduction.unapply(config.motionTargets.angularJerk(config.shaftDiameter)))
        }

        motorController.clearStickyFaults()
        motorController.configurator.apply(talonConfig)
     }

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current climber elevator displacement (meters)", { displacement.`in`(Meters) }, {})
            addDoubleProperty("Inverse operation (rotations)", { motorPosition.`in`(Rotations) }, {})
        }
    }

    fun coast(): Command = Commands.runOnce({
        motorController.setNeutralMode(NeutralModeValue.Coast)
    })

    fun brake(): Command = Commands.runOnce({
        motorController.setNeutralMode(NeutralModeValue.Brake)
    })
}
