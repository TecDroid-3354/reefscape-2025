package net.tecdroid.subsystems.wrist

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import net.tecdroid.util.*
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface WristIO {
    fun updateInputs(inputs: WristIOInputs) {}

    fun setVoltage(voltage: Voltage) {}
    fun setAngle(angle: Angle) {}
    fun setMotorPosition(angle: Angle) {}
    fun setTargetAngle(angle: Angle) {}

    fun getTargetAngle(): Angle {return Degrees.zero()}
    fun getAbsoluteEncoderInstance(): ThroughBoreAbsoluteEncoder {
        return ThroughBoreAbsoluteEncoder(NumericId(0), 0.0.degrees, false)
    }

    // Must override during implementation. Just to fill TdSubsystem information. Probably will change later.
    fun getMotorPosition(): Angle {return Degrees.zero()}
    fun getMotorVelocity(): AngularVelocity {return DegreesPerSecond.zero()}
    fun getMotorPower(): Double {return 0.0}

    fun coast() {}
    fun brake() {}

    /**
     * Intended to store all relevant inputs for the Wrist. These inputs would be shown during replay / simulation
     * in AdvantageScope for every cycle.
     * TODO() = Use @AutoLog instead of implementing LoggableInputs, same but deprecated.
     * @property isThroughBoreConnected Indicates whether the throughbore is connected in this cycle.
     * @property throughBoreAbsolutePosition Absolute position reported by the throughbore in this cycle.
     * @property isMotorConnected Indicates whether the motor is connected in this cycle.
     * @property motorPosition Position reported by the motor in this cycle.
     * @property motorVoltage Voltage of the motor in this cycle.
     * @property motorSupplyCurrent Supply current of the motor in this cycle.
     * @property motorTemperature Temperature of the motor in this cycle.
     * @property wristTargetAngle Desired angle for the wrist in this cycle.
     * @property wristAngle Angle of the wrist in this cycle.
     */
    class WristIOInputs: LoggableInputs {
        var isThroughBoreConnected: Boolean = false
        var throughBoreAbsolutePosition: Angle = Degrees.zero()
        var isMotorConnected: Boolean = false
        var motorPosition: Angle = Degrees.zero()
        var motorVoltage: Voltage = 0.0.volts
        var motorSupplyCurrent: Current = 0.0.amps
        var motorTemperature: Temperature = 0.0.degreesCelsius
        var wristTargetAngle: Angle = Degrees.zero()
        var wristAngle: Angle = Degrees.zero()

        override fun toLog(table: LogTable) {
            table.put("IsThroughBoreConnected", isThroughBoreConnected)
            table.put("ThroughBoreAbsolutePosition", throughBoreAbsolutePosition)
            table.put("IsMotorConnected", isMotorConnected)
            table.put("MotorPosition", motorPosition)
            table.put("MotorVoltage", motorVoltage)
            table.put("MotorSupplyCurrent", motorSupplyCurrent)
            table.put("MotorTemperature", motorTemperature)
            table.put("WristTargetAngle", wristTargetAngle)
            table.put("WristAngle", wristAngle)
        }

        override fun fromLog(table: LogTable) {
            isThroughBoreConnected = table.get("IsThroughboreConnected").boolean
            throughBoreAbsolutePosition = table.get("ThroughboreAbsolutePosiiton").double.degrees
            isMotorConnected = table.get("IsMotorConnected").boolean
            motorPosition = table.get("MotorPosition").double.degrees
            motorVoltage = table.get("MotorVoltage").double.volts
            motorSupplyCurrent = table.get("MotorSupplyCurrent").double.amps
            motorTemperature = table.get("MotorTemperature").double.degreesCelsius
            wristTargetAngle = table.get("WristTargetAngle").double.degrees
            wristAngle = table.get("WristAngle").double.degrees
        }
    }
}