package net.tecdroid.subsystems.elevator

import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.*
import net.tecdroid.util.*
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ElevatorIO {
    fun updateInputs(inputs: ElevatorIOInputs) {}

    fun setVoltage(voltage: Voltage) {}
    fun setAngularDisplacement(motorDisplacement: Angle) {}

    fun setTargetAngle(angle: Angle) {}
    fun getTargetAngle(): Angle {return 0.0.degrees}

    fun getMotorPosition(): Angle {return 0.0.degrees}
    fun getMotorVelocity(): AngularVelocity {return 0.0.degrees.per(Second)}
    fun getMotorPower(): Double {return 0.0}

    fun coast() {}
    fun brake() {}

    @AutoLog
    class ElevatorIOInputs: LoggableInputs {
        var isLeadMotorConnected: Boolean = false
        var isFollowerMotorConnected: Boolean = false
        var leadMotorPosition: Angle = 0.0.degrees
        var followerMotorPosition: Angle = 0.0.degrees
        var leadMotorVoltage: Voltage = 0.0.volts
        var followerMotorVoltage: Voltage = 0.0.volts
        var leadMotorSupplyCurrent: Current = 0.0.amps
        var followerMotorSupplyCurrent: Current = 0.0.amps
        var leadMotorTemperature : Temperature = 0.0.degreesCelsius
        var followerMotorTemperature: Temperature = 0.0.degreesCelsius
        var elevatorTargetDisplacement: Distance = 0.0.meters
        var elevatorDisplacement: Distance = 0.0.meters

        override fun toLog(table: LogTable) {
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
            table.put("ElevatorTargetDisplacement", elevatorTargetDisplacement)
            table.put("ElevatorDisplacement", elevatorDisplacement)
        }

        override fun fromLog(table: LogTable) {
            isLeadMotorConnected = table.get("IsLeadMotorConnected") as Boolean
            isFollowerMotorConnected = table.get("IsFollowerMotorConnected") as Boolean
            leadMotorPosition = table.get("LeadMotorPosition") as Angle
            followerMotorPosition = table.get("FollowerMotorPosition") as Angle
            leadMotorVoltage = table.get("LeadMotorVoltage") as Voltage
            followerMotorVoltage = table.get("FollowerMotorVoltage") as Voltage
            leadMotorTemperature = table.get("LeadMotorTemperature") as Temperature
            followerMotorTemperature = table.get("FollowerMotorTemperature") as Temperature
            elevatorTargetDisplacement = table.get("ElevatorTargetDisplacement") as Distance
            elevatorDisplacement = table.get("ElevatorDisplacement") as Distance
        }
    }

}