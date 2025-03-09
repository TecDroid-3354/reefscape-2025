package net.tecdroid.subsystems.intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.subsystems.util.generic.VoltageControlledSubsystem

class Intake(private val config: IntakeConfig) : SubsystemBase(), VoltageControlledSubsystem {
    private val motorController = TalonFX(config.motorControllerId.id)

    init {
        configureMotorInterface()
    }

    // ////////////// //
    // INTAKE CONTROL //
    // ////////////// //

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        motorController.setControl(request)
    }

    // ///////////// //
    // Configuration //
    // ///////////// //

    private fun configureMotorInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)

            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.currentLimit)
        }


        motorController.clearStickyFaults()
        motorController.configurator.apply(talonConfig)
    }
}
