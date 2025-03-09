package net.tecdroid.subsystems.elevatorjoint

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Time
import net.tecdroid.util.*
import net.tecdroid.util.Motors.krakenX60

data class ElevatorConfig(
    val leftMotorId: NumericId,
    val rightMotorId: NumericId,
    val absoluteEncoderPort: NumericId,
    val motorProperties: MotorProperties,
    val currentLimit: Current,
    val absoluteMinimumAngle: Angle,
    val absoluteMaximumAngle: Angle,
    val minimumAngle: Angle,
    val maximumAngle: Angle,
)

val elevatorConfig = ElevatorConfig(

)

class ElevatorJointConfiguration {
    private class DeviceIDS {
        private val mElevatorAngleDigit = DigitId(5)
        private val mLeadingMotorDigit = DigitId(1)
        private val mFollowerMotorDigit = DigitId(2)

        // Pending: Throughbore port
        private val TroughBorePort = DigitId(3)

        val elevatorAngleDevicesIDs: ElevatorJoint.DeviceIdentifiers = ElevatorJoint.DeviceIdentifiers(
            joinDigits(mElevatorAngleDigit, mLeadingMotorDigit),
            joinDigits(mElevatorAngleDigit, mFollowerMotorDigit),
            joinDigits(mElevatorAngleDigit, TroughBorePort)
        )
    }

    private class Devices {
        private val elevatorAngleMotorProperties = krakenX60

        val elevatorAngleDeviceProperties: ElevatorJoint.DeviceProperties = ElevatorJoint.DeviceProperties(
            elevatorAngleMotorProperties
        )
    }

    private class Limits {
        private val motorsCurrent: Current = Units.Amps.of(40.0)
        private val minimumElevatorAngle: Angle = Units.Degrees.of(10.0)
        private val maximumElevatorAngle: Angle = Units.Degrees.of(90.0)

        val elevatorAngleDeviceLimits: ElevatorJoint.DeviceLimits = ElevatorJoint.DeviceLimits(
            motorsCurrent, minimumElevatorAngle, maximumElevatorAngle
        )
    }

    private class Conventions {
        private val mElevatorAngleMotorsPositiveDirection = RotationalDirection.Clockwise

        val elevatorAngleDeviceConventions: ElevatorJoint.DeviceConventions = ElevatorJoint.DeviceConventions(
            mElevatorAngleMotorsPositiveDirection
        )
    }

    private class Structure {
        private val elevatorAngleMotorGR = GearRatio(150.0, 1.0, 0)
        private val encoderOffset: Angle = Units.Degrees.of(0.0)
        val elevatorAnglePhysicalDescription: ElevatorJoint.PhysicalDescription = ElevatorJoint.PhysicalDescription(
            elevatorAngleMotorGR, encoderOffset
        )
    }

    private class Control {
        private val elevatorAnglePidfCoefficients = PidfCoefficients(0.0, 0.0, 0.0, 0.0)
        private val elevatorAngleSvagGains = SvagGains(0.0, 0.0, 0.0, 0.0)
        private val elevatorAngleMotionMagicTargets = MotionMagicTargets(0.0, 0.0, 0.0)
        private val elevatorAngleRampRate: Time = Units.Seconds.of(0.1)

        val elevatorAngleControlConstants: ElevatorJoint.ControlConstants = ElevatorJoint.ControlConstants(
            elevatorAnglePidfCoefficients, elevatorAngleSvagGains,
            elevatorAngleMotionMagicTargets, elevatorAngleRampRate
        )
    }

    val elevatorAngleConfig: ElevatorJoint.Config = ElevatorJoint.Config(
        DeviceIDS().elevatorAngleDevicesIDs, Devices().elevatorAngleDeviceProperties,
        Limits().elevatorAngleDeviceLimits, Conventions().elevatorAngleDeviceConventions,
        Structure().elevatorAnglePhysicalDescription, Control().elevatorAngleControlConstants
    )
}


