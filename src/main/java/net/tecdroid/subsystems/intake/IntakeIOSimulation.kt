package net.tecdroid.subsystems.intake

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import net.tecdroid.core.Robot
import net.tecdroid.util.amps
import net.tecdroid.util.milliseconds
import net.tecdroid.util.volts

class IntakeIOSimulation: IntakeIO {
    /**
     * Instantiates a simulated motor controller.
     */
    private val motorController = DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.004, 1.0),
        DCMotor.getKrakenX60(1)
    )

    private var appliedVolts = 0.0.volts

    override fun setVoltage(voltage: Voltage) {
        appliedVolts = MathUtil.clamp(voltage.`in`(Volts), -12.0, 12.0).volts
    }

    /**
     * Updates the inputs during simulation.
     * Properties isMotorConnected and motorTemperature are not updated during simulation.
     */
    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
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