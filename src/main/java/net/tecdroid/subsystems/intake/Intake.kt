package net.tecdroid.subsystems.intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.subsystems.util.generic.TdSubsystem
import java.sql.Driver

class Intake(private val sensor : DigitalInput) : TdSubsystem("Intake") {
    private val config = intakeConfig
    private val motorController = TalonFX(config.motorControllerId.id)
    private val trigger = Trigger(sensor::get)

    override val forwardsRunningCondition = { true }
    override val backwardsRunningCondition = { true }

    override val motorPosition: Angle
        get() = motorController.position.value

    override val motorVelocity: AngularVelocity
        get() = motorController.velocity.value

    override val power: Double
        get() = motorController.get()

    init {
        configureMotorInterface()
        (trigger.negate()).and { DriverStation.isTeleop() }.onTrue(stopCommand())
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        motorController.setControl(request)
    }

    fun hasCoral() = !sensor.get()

    private fun configureMotorInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)

            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.motorCurrentLimit)
        }


        motorController.clearStickyFaults()
        motorController.configurator.apply(talonConfig)
    }
}
