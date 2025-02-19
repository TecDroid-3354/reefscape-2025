package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.subsystems.drivetrain.SwerveModule.*
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Counterclockwise
import net.tecdroid.util.geometry.Wheel

object SwerveModuleConfiguration {
    internal object DeviceIds {
        private val driveControllerDigit = DigitId(1)
        private val steerControllerDigit = DigitId(2)
        private val absoluteEncoderDigit = DigitId(3)

        internal val deviceIdentifiers: List<DeviceIdentifiers> = (1..4).map { DigitId(it) }.map {
            DeviceIdentifiers(
                joinDigits(it, driveControllerDigit),
                joinDigits(it, steerControllerDigit),
                joinDigits(it, absoluteEncoderDigit)
            )
        }
    }

    internal object Structure {
        private val driveGearRatio = GearRatio(6.12, 1.0, 4)
        private val steerGearRatio = GearRatio(150.0, 7.0, 2)

        private val wheel = Wheel.fromRadius(Inches.of(2.0))

        internal val physicalDescription = PhysicalDescription(driveGearRatio, steerGearRatio, wheel)
    }

    internal object Conventions {
        private val driveWheelPositiveDirection = Counterclockwise;
        private val steerAzimuthPositiveDirection = Counterclockwise;

        internal val deviceConventions = DeviceConventions(driveWheelPositiveDirection, steerAzimuthPositiveDirection)
    }

    internal object Specifics {
        private val magnetOffsets: List<Angle> = listOf(
            Rotations.of(-0.09130859375),
            Rotations.of(-0.38982578125),
            Rotations.of(-0.345458984375),
            Rotations.of(+0.138427734375)
        )

        internal val moduleSpecifics = magnetOffsets.map { ModuleSpecifics(it) }
    }

    internal object Devices {
        private val driveMotorProperties = Motors.krakenX60
        private val steerMotorProperties = Motors.neo

        internal val deviceProperties = DeviceProperties(driveMotorProperties, steerMotorProperties)
    }

    internal object Limits {
        private val driveMotorCurrentLimit: Current = Amps.of(40.0)
        private val steerMotorCurrentLimit: Current = Amps.of(30.0)

        internal val deviceLimits = DeviceLimits(driveMotorCurrentLimit, steerMotorCurrentLimit)
    }

    internal object Control {
        private val drivePidf: PidfCoefficients = PidfCoefficients(0.0, 0.0, 0.00, 0.0)
        private val driveSvag: SvagGains = SvagGains(0.132, 0.12, 0.01, 0.0)

        private val steerPidf: PidfCoefficients = PidfCoefficients(0.1, 0.0, 0.01, 0.0)
        private val steerSvag: SvagGains = SvagGains(0.0, 0.0, 0.0, 0.0)

        internal val controlConstants = ControlConstants(drivePidf, driveSvag, steerPidf, steerSvag)
    }

    val configurations =
        DeviceIds.deviceIdentifiers.zip(Specifics.moduleSpecifics).map { (deviceIdentifier, specifics) ->
            Config(
                deviceIdentifier,
                Devices.deviceProperties,
                Conventions.deviceConventions,
                Limits.deviceLimits,
                Structure.physicalDescription,
                Control.controlConstants,
                specifics
            )
        }

}
