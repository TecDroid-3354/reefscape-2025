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
 * Intended to store all relevant inputs for the Intake. These inputs would be shown during replay / simulation
 * in AdvantageScope for every cycle.
 * @property isMotorConnected Indicates whether the motor is connected in this cycle.
 * @property isSensorTriggered Indicates if there's a coral inside the intake.
 * @property motorVoltage Voltage of the motor in this cycle.
 * @property motorSupplyCurrent Supply current of the motor in this cycle.
 * @property motorTemperature Temperature of the motor in this cycle.
 */
@AutoLog
open class IntakeIOInputs {
    @JvmField var isMotorConnected: Boolean = true
    @JvmField var isSensorTriggered: Boolean = false
    @JvmField var motorVoltage: Voltage = 0.0.volts
    @JvmField var motorSupplyCurrent: Current = 0.0.amps
    @JvmField var motorTemperature: Temperature = 10.0.degreesCelsius
}

/**
 * Input / Output layer for the intake. This needs to be implemented for both the real robot and the simulations.
 */
interface IntakeIO {
    fun updateInputs(inputs: IntakeIOInputs) {}         // Must override during implementation

    fun setVoltage(voltage: Voltage) {}                 // Must override during implementation
    fun isSensorTriggered(): Boolean {return false}     // Must override during implementation

    // Must override during implementation. Just to fill TdSubsystem information. Probably will change later.
    fun getMotorPosition(): Angle {return Degrees.zero()}
    fun getMotorVelocity(): AngularVelocity {return DegreesPerSecond.zero()}
    fun getMotorPower(): Double {return 0.0}
}
