package net.tecdroid.subsystems.climber.climberIntake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.subsystems.util.generic.TdSubsystem
import net.tecdroid.util.volts

class ClimberIntake(private val config: ClimberIntakeConfig) : TdSubsystem("ClimberIntake") {
    private val motorController = TalonFX(config.motorControllerId.id)

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
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        motorController.setControl(request)
    }

    fun enableFor(voltage: Voltage, time: Time) = setVoltageCommand { voltage }
                                                . withTimeout(time)
                                                . andThen(setVoltageCommand { 0.0.volts })

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

    fun coast(): Command = Commands.runOnce({
        motorController.setNeutralMode(NeutralModeValue.Coast)
    })

    fun brake(): Command = Commands.runOnce({
        motorController.setNeutralMode(NeutralModeValue.Brake)
    })
}
