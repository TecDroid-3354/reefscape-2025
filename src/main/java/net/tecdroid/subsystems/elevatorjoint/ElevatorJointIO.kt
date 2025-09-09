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

@AutoLog
open class ElevatorJointIOInputs {
    @JvmField var isThroughBoreConnected: Boolean = false
    @JvmField var throughBoreAbsolutePosition: Angle = 0.0.degrees
    @JvmField var isLeadMotorConnected: Boolean = false
    @JvmField var isFollowerMotorConnected: Boolean = false
    @JvmField var leadMotorPosition: Angle = 0.0.degrees
    @JvmField var followerMotorPosition: Angle = 0.0.degrees
    @JvmField var leadMotorVoltage: Voltage = 0.0.volts
    @JvmField var followerMotorVoltage: Voltage = 0.0.volts
    @JvmField var leadMotorSupplyCurrent: Current = 0.0.amps
    @JvmField var followerMotorSupplyCurrent: Current = 0.0.amps
    @JvmField var leadMotorTemperature: Temperature = 0.0.degreesCelsius
    @JvmField var followerMotorTemperature: Temperature = 0.0.degreesCelsius
    @JvmField var elevatorJointTargetAngle: Angle = 0.0.degrees
    @JvmField var elevatorJointAngle: Angle = 0.0.degrees
}

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


}