package net.tecdroid.subsystems.elevatorjoint

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import net.tecdroid.util.*
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ElevatorJointIO {
    fun updateInputs(inputs: ElevatorJointIOInputs) {}

    fun setVoltage(voltage: Voltage) {}
    fun setAngle(angle: Angle) {}
    fun setMotorPosition(position: Angle) {}
    fun setTargetAngle(angle: Angle) {}

    fun getTargetAngle(): Angle {return Degrees.zero()}
    fun getAbsoluteEncoderInstance(): ThroughBoreAbsoluteEncoder {
        return ThroughBoreAbsoluteEncoder(NumericId(0), 0.0.degrees, false)
    }

    fun getMotorPosition(): Angle {return Degrees.zero()}
    fun getMotorVelocity(): AngularVelocity {return DegreesPerSecond.zero()}
    fun getMotorPower(): Double {return 0.0}

    fun coast() {}
    fun brake() {}

    @AutoLog
    class ElevatorJointIOInputs: LoggableInputs {
        var isThroughBoreConnected: Boolean = false
        var throughBoreAbsolutePosition: Angle = 0.0.degrees
        var isLeadMotorConnected: Boolean = false
        var isFollowerMotorConnected: Boolean = false
        var leadMotorPosition: Angle = 0.0.degrees
        var followerMotorPosition: Angle = 0.0.degrees
        var leadMotorVoltage: Voltage = 0.0.volts
        var followerMotorVoltage: Voltage = 0.0.volts
        var leadMotorSupplyCurrent: Current = 0.0.amps
        var followerMotorSupplyCurrent: Current = 0.0.amps
        var leadMotorTemperature: Temperature = 0.0.degreesCelsius
        var followerMotorTemperature: Temperature = 0.0.degreesCelsius
        var elevatorJointTargetAngle: Angle = 0.0.degrees
        var elevatorJointAngle: Angle = 0.0.degrees

        override fun toLog(table: LogTable) {
            table.put("IsThroughBoreConnected", isThroughBoreConnected)
            table.put("ThroughBoreAbsolutePosition", throughBoreAbsolutePosition)
            table.put("IsLeadMotorConnected", isLeadMotorConnected)
            table.put("IsFollowerMotorConnected", isFollowerMotorConnected)
            table.put("LeadMotorPosition", leadMotorPosition)
            table.put("FollowerMotorPosition", followerMotorPosition)
            table.put("LeadMotorVoltage", leadMotorVoltage)
            table.put("FollowerMotorVoltage", followerMotorVoltage)
            table.put("LeadMotorSupplyCurrent", leadMotorSupplyCurrent)
            table.put("FollowerMotorSupplyCurrent", followerMotorSupplyCurrent)
            table.put("LeadMotorTemperature", leadMotorTemperature)
            table.put("FollowerMotorTemperature", followerMotorTemperature)
            table.put("ElevatorJointTargetAngle", elevatorJointTargetAngle)
            table.put("ElevatorJointAngle", elevatorJointAngle)
        }

        override fun fromLog(table: LogTable) {
            isThroughBoreConnected = table.get("IsThroughBoreConnected") as Boolean
            throughBoreAbsolutePosition = table.get("ThroughBoreAbsolutePosition") as Angle
            isLeadMotorConnected = table.get("IsLeadMotorConnected") as Boolean
            isFollowerMotorConnected = table.get("IsFollowerMotorConnected") as Boolean
            leadMotorPosition = table.get("LeadMotorPosition") as Angle
            followerMotorPosition = table.get("FollowerMotorPosition") as Angle
            leadMotorVoltage = table.get("LeadMotorVoltage") as Voltage
            followerMotorVoltage = table.get("FollowerMotorVoltage") as Voltage
            leadMotorSupplyCurrent = table.get("LeadMotorSupplyCurrent") as Current
            followerMotorSupplyCurrent = table.get("FollowerMotorSupplyCurrent") as Current
            leadMotorTemperature = table.get("LeadMotorTemperature") as Temperature
            followerMotorTemperature = table.get("FollowerMotorTemperature") as Temperature
            elevatorJointTargetAngle = table.get("ElevatorJointTargetAngle") as Angle
            elevatorJointAngle = table.get("ElevatorJointAngle") as Angle
        }
    }
}