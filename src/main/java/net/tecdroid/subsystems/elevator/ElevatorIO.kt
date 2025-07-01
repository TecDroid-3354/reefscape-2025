package net.tecdroid.subsystems.elevator

import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.*
import net.tecdroid.util.*
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

@AutoLog
open class ElevatorIOInputs {
    @JvmField var isLeadMotorConnected: Boolean = false
    @JvmField var isFollowerMotorConnected: Boolean = false
    @JvmField var leadMotorPosition: Angle = 0.0.degrees
    @JvmField var followerMotorPosition: Angle = 0.0.degrees
    @JvmField var leadMotorVoltage: Voltage = 0.0.volts
    @JvmField var followerMotorVoltage: Voltage = 0.0.volts
    @JvmField var leadMotorSupplyCurrent: Current = 0.0.amps
    @JvmField var followerMotorSupplyCurrent: Current = 0.0.amps
    @JvmField var leadMotorTemperature : Temperature = 0.0.degreesCelsius
    @JvmField var followerMotorTemperature: Temperature = 0.0.degreesCelsius
    @JvmField var elevatorTargetDisplacement: Distance = 0.0.meters
    @JvmField var elevatorDisplacement: Distance = 0.0.meters
}

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
}