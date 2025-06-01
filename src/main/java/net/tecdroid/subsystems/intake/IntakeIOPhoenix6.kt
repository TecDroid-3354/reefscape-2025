package net.tecdroid.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import net.tecdroid.util.hertz
import net.tecdroid.util.volts

/**
 * Intake's IO interface layer adapted for Phoenix6 (TalonFX).
 * @param config Desired configuration of the intake. Must use Phoenix6.
 */
class IntakeIOPhoenix6(private val config: IntakeConfig): IntakeIO {
    private val motorController = TalonFX(config.motorControllerId.id)
    // Properties stored individually as signals to give them an update frequency different from the controller's.
    private val motorVoltage: StatusSignal<Voltage> = motorController.motorVoltage
    private val motorSupplyCurrent: StatusSignal<Current> = motorController.supplyCurrent
    private val motorTemperature: StatusSignal<Temperature> = motorController.deviceTemp

    // Voltage request initialized here to avoid creating new objects every call.
    private var voltageRequest = VoltageOut(0.0.volts)

    /**
     * Called after the primary constructor. Used to configure intake's motor and update frequencies.
     */
    init {
        configureMotorInterface()
        // Reduces the update frequency from the default 100hz to 50hz to match the periodic calls.
        BaseStatusSignal.setUpdateFrequencyForAll(50.0.hertz, motorVoltage, motorSupplyCurrent, motorTemperature)
        motorController.optimizeBusUtilization() // Refreshes every 100ms.
    }

    /**
     * Sends a VoltageOut request to the motor controller. Excessive voltage will be clamped within range.
     * @param voltage Desired voltage. Accepted range: [[-12V, 12V]]
     */
    override fun setVoltage(voltage: Voltage) {
        val clampedVoltage = MathUtil.clamp(voltage.`in`(Units.Volts), 12.0.unaryMinus(), 12.0).volts
        motorController.setControl(voltageRequest.withOutput(clampedVoltage))
    }

    /**
     * Must be called periodically. Updates all relevant inputs reported by the motor at this timestamp.
     * @param inputs Initialized inputs object used across the subsystem.
     */
    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.isMotorConnected = BaseStatusSignal.refreshAll(
            motorVoltage,
            motorSupplyCurrent,
            motorTemperature
        ).isOK // If every signal refreshed successfully, the motor must be connected.

        inputs.motorVoltage = motorVoltage.value
        inputs.motorSupplyCurrent = motorSupplyCurrent.value
        inputs.motorTemperature = motorTemperature.value
    }

    override fun getMotorPosition(): Angle { return motorController.position.value }
    override fun getMotorVelocity(): AngularVelocity { return motorController.velocity.value }
    override fun getMotorPower(): Double { return motorController.motorVoltage.valueAsDouble.div(12.0) }


    /**
     * Takes care of the motor configuration.
     * TODO(): Generalize motor configuration within the project.
     */
    private fun configureMotorInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(config.motorNeutralMode)

            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.motorCurrentLimit)
        }

        motorController.clearStickyFaults()
        motorController.configurator.apply(talonConfig)
    }
}