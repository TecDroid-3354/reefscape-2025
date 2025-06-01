package net.tecdroid.subsystems.intake

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import net.tecdroid.util.amps
import net.tecdroid.util.degreesCelsius
import net.tecdroid.util.volts
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * Input / Output layer for the intake. This needs to be implemented for both the real robot and the simulations.
 */
interface IntakeIO {

    fun setVoltage(voltage: Voltage) {}                 // Must override during implementation
    fun isSensorTriggered(): Boolean {return false}     // Must override during implementation
    fun updateInputs(inputs: IntakeIOInputs) {}         // Must override during implementation

    // Must override during implementation. Just to fill TdSubsystem information. Probably will change later.
    fun getMotorPosition(): Angle {return Degrees.zero()}
    fun getMotorVelocity(): AngularVelocity {return DegreesPerSecond.zero()}
    fun getMotorPower(): Double {return 0.0}

    /**
     * Intended to store all relevant inputs for the Intake. These inputs would be shown during replay / simulation
     * in AdvantageScope for every cycle.
     * TODO() = Use @AutoLog instead of implementing LoggableInputs, same but deprecated.
     * @property isMotorConnected Indicates whether the motor is connected in this cycle.
     * @property isSensorTriggered Indicates if there's a coral inside the intake.
     * @property motorVoltage Voltage of the motor in this cycle.
     * @property motorSupplyCurrent Supply current of the motor in this cycle.
     * @property motorTemperature Temperature of the motor in this cycle.
     */
    class IntakeIOInputs: LoggableInputs {
        var isMotorConnected: Boolean = true
        var isSensorTriggered: Boolean = false
        var motorVoltage: Voltage = 0.0.volts
        var motorSupplyCurrent: Current = 0.0.amps
        var motorTemperature: Temperature = 10.0.degreesCelsius

        override fun toLog(table: LogTable) {
            table.put("IsMotorConnected", isMotorConnected)
            table.put("IsSensorTriggered", isSensorTriggered)
            table.put("MotorVoltage", motorVoltage)
            table.put("MotorSupplyCurrent", motorSupplyCurrent)
            table.put("MotorTemperature", motorTemperature)
        }

        override fun fromLog(table: LogTable) {
            isMotorConnected = table.get("IsMotorConnected").boolean
            isSensorTriggered = table.get("isSensorTriggered").boolean
            motorVoltage = table.get("MotorVoltage").double.volts
            motorSupplyCurrent = table.get("MotorSupplyCurrent").double.amps
            motorTemperature = table.get("MotorTemperature").double.degreesCelsius
        }
    }
}