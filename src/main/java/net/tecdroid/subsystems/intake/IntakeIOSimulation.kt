package net.tecdroid.subsystems.intake

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import net.tecdroid.util.amps
import net.tecdroid.util.volts

/**
 * [IntakeIO] interface layer adapted for simulation.
 * Sensor is not taken into account, not sure how would that affect the simulation. Need to test.
 */
class IntakeIOSimulation: IntakeIO {
    /**
     * Instantiates a simulated motor controller. In the [LinearSystemId], 2nd & 3rd parameters are invented.
     */
    private val motorController = DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.5, 1.0),
        DCMotor.getKrakenX60(1)
    )

    private var appliedVolts = 0.0.volts

    /**
     * Updates the [appliedVolts] of this class, which will be sent to the simulated motor in the next cycle.
     * @param voltage Desired voltage. this value will be clamped within [[-12V, 12V]]
     */
    override fun setVoltage(voltage: Voltage) {
        appliedVolts = MathUtil.clamp(voltage.`in`(Volts), -12.0, 12.0).volts
    }

    /**
     * Updates the inputs during simulation. The desired [appliedVolts] are passed to the motor controller here
     * to ensure those are updated before the motor controller's update.
     * Properties isMotorConnected and motorTemperature are not updated during simulation.
     * @param inputs Initialized inputs object used within the subsystem.
     */
    override fun updateInputs(inputs: IntakeIOInputs) {
        motorController.inputVoltage = appliedVolts.`in`(Volts)
        motorController.update(0.02) // 2ms is the default robot periodic time.

        inputs.motorVoltage = motorController.inputVoltage.volts
        inputs.motorSupplyCurrent = motorController.currentDrawAmps.amps
    }

    override fun getMotorPosition(): Angle {
        return motorController.angularPosition
    }

    override fun getMotorVelocity(): AngularVelocity {
        return motorController.angularVelocity
    }

    override fun getMotorPower(): Double {
        return motorController.inputVoltage.div(12.0)
    }
}