package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Time
import net.tecdroid.constants.RobotSpatialConvention
import net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.ModuleConfig.moduleConfigurations
import net.tecdroid.subsystems.drivetrain.SwerveModule.*
import net.tecdroid.util.*
import net.tecdroid.util.geometry.Square
import net.tecdroid.util.geometry.Wheel
import net.tecdroid.util.geometry.Wheel.Companion.fromRadius

object SwerveDriveConstants {
    private val imuId: CanId = CanId(1)

    // Drivetrain Measurements //
    val drivetrainDimensions = Square(Inches.of(22.3))
    val quarterDrivetrain = drivetrainDimensions.scale(0.5)

    // Module Characteristics //
    val wheel: Wheel = fromRadius(Inches.of(2.0))

    val driveGearRatio: GearRatio = GearRatio(6.12, 1.0)
    val steerGearRatio: GearRatio = GearRatio(150.0, 7.0)

    val driveMotorRotationalConvention = RotationalConvention.counterclockwise()
    val steerMotorRotationalConvention = RotationalConvention.counterclockwise()

    // Limits //
    val driveMotorCurrentLimit: Current = Units.Amps.of(40.0)
    val steerMotorCurrentLimit: Current = Units.Amps.of(30.0)

    val driveMotorRampRate: Time = Units.Seconds.of(0.1)

    internal object Ids {
        private val frontRightModule: DigitId = DigitId(1)
        private val frontLeftModule: DigitId = DigitId(2)
        private val backLeftModule: DigitId = DigitId(3)
        private val backRightModule: DigitId = DigitId(4)

        private val driveControllerDigit: DigitId = DigitId(1)
        private val steerControllerDigit: DigitId = DigitId(2)
        private val absoluteEncoderDigit: DigitId = DigitId(3)

        val frontRightDriveController: CanId = joinDigits(frontRightModule, driveControllerDigit).toCanId()
        val frontLeftDriveController: CanId = joinDigits(frontLeftModule, driveControllerDigit).toCanId()
        val backLeftDriveController: CanId = joinDigits(backLeftModule, driveControllerDigit).toCanId()
        val backRightDriveController: CanId = joinDigits(backRightModule, driveControllerDigit).toCanId()

        val frontRightSteerController: CanId = joinDigits(frontRightModule, steerControllerDigit).toCanId()
        val frontLeftSteerController: CanId = joinDigits(frontLeftModule, steerControllerDigit).toCanId()
        val backLeftSteerController: CanId = joinDigits(backLeftModule, steerControllerDigit).toCanId()
        val backRightSteerController: CanId = joinDigits(backRightModule, steerControllerDigit).toCanId()

        val frontRightAbsoluteEncoder: CanId = joinDigits(frontRightModule, absoluteEncoderDigit).toCanId()
        val frontLeftAbsoluteEncoder: CanId = joinDigits(frontLeftModule, absoluteEncoderDigit).toCanId()
        val backLeftAbsoluteEncoder: CanId = joinDigits(backLeftModule, absoluteEncoderDigit).toCanId()
        val backRightAbsoluteEncoder: CanId = joinDigits(backRightModule, absoluteEncoderDigit).toCanId()
    }

    internal object ModuleState {
        val frontRightMagnetOffset: Angle = Units.Rotations.of(-0.09130859375)
        val frontLeftMagnetOffset: Angle = Units.Rotations.of(-0.38982578125)
        val backLeftMagnetOffset: Angle = Units.Rotations.of(-0.345458984375)
        val backRightMagnetOffset: Angle = Units.Rotations.of(+0.138427734375)

        val frontRightModuleOffset: Translation2d = Translation2d(
            RobotSpatialConvention.longitudinal.front(quarterDrivetrain.length),
            RobotSpatialConvention.transversal.right(quarterDrivetrain.width)
        )

        val frontLeftModuleOffset: Translation2d = Translation2d(
            RobotSpatialConvention.longitudinal.front(quarterDrivetrain.length),
            RobotSpatialConvention.transversal.left(quarterDrivetrain.width)
        )

        val backLeftModuleOffset: Translation2d = Translation2d(
            RobotSpatialConvention.longitudinal.back(quarterDrivetrain.length),
            RobotSpatialConvention.transversal.left(quarterDrivetrain.width)
        )

        val backRightModuleOffset: Translation2d = Translation2d(
            RobotSpatialConvention.longitudinal.back(quarterDrivetrain.length),
            RobotSpatialConvention.transversal.right(quarterDrivetrain.width)
        )
    }

    internal object ControlConstants {
        val drivePidf: PidfCoefficients = PidfCoefficients(0.001 * 0.0, 0.0, 0.00, 0.0)
        val driveSvag: SvagGains = SvagGains(0.132, 0.12, 0.01, 0.0)

        val steerPidf: PidfCoefficients = PidfCoefficients(0.01, 0.0, 0.001, 0.0)
        val steerSvag: SvagGains = SvagGains(0.0, 0.0, 0.0, 0.0)
    }

    internal object ModuleConfig {
        private val frontRightIdConfig: IdentifierConfig = IdentifierConfig(
            Ids.frontRightDriveController,
            Ids.frontRightSteerController,
            Ids.frontRightAbsoluteEncoder
        )

        private val frontLeftIdConfig: IdentifierConfig = IdentifierConfig(
            Ids.frontLeftDriveController,
            Ids.frontLeftSteerController,
            Ids.frontLeftAbsoluteEncoder
        )

        private val backLeftIdConfig: IdentifierConfig = IdentifierConfig(
            Ids.backLeftDriveController,
            Ids.backLeftSteerController,
            Ids.backLeftAbsoluteEncoder
        )

        private val backRightIdConfig: IdentifierConfig = IdentifierConfig(
            Ids.backRightDriveController,
            Ids.backRightSteerController,
            Ids.backRightAbsoluteEncoder
        )

        private val frontRightStateConfig: StateConfig = StateConfig(
            ModuleState.frontRightMagnetOffset,
            RotationalDirection.Counterclockwise,
            RotationalDirection.Counterclockwise
        )
        private val frontLeftStateConfig: StateConfig = StateConfig(
            ModuleState.frontLeftMagnetOffset,
            RotationalDirection.Counterclockwise,
            RotationalDirection.Counterclockwise
        )
        private val backLeftStateConfig: StateConfig = StateConfig(
            ModuleState.backLeftMagnetOffset,
            RotationalDirection.Counterclockwise,
            RotationalDirection.Counterclockwise
        )
        private val backRightStateConfig: StateConfig = StateConfig(
            ModuleState.backRightMagnetOffset,
            RotationalDirection.Counterclockwise,
            RotationalDirection.Counterclockwise
        )

        private val frontRightPhysicalDescription: PhysicalDescription =
            PhysicalDescription(ModuleState.frontRightModuleOffset, driveGearRatio, steerGearRatio, wheel)

        private val frontLeftPhysicalDescription: PhysicalDescription =
            PhysicalDescription(ModuleState.frontLeftModuleOffset, driveGearRatio, steerGearRatio, wheel)

        private val backLeftPhysicalDescription: PhysicalDescription =
            PhysicalDescription(ModuleState.backLeftModuleOffset, driveGearRatio, steerGearRatio, wheel)

        private val backRightPhysicalDescription: PhysicalDescription =
            PhysicalDescription(ModuleState.backRightModuleOffset, driveGearRatio, steerGearRatio, wheel)

        private val moduleControlConfig: ControlConfig = ControlConfig(
            ControlConstants.drivePidf,
            ControlConstants.driveSvag,
            ControlConstants.steerPidf,
            ControlConstants.steerSvag
        )

        private val moduleLimitConfig: LimitConfig =
            LimitConfig(driveMotorCurrentLimit, driveMotorRampRate, steerMotorCurrentLimit)

        private val frontRight: Config = Config(
            frontRightIdConfig,
            frontRightStateConfig,
            frontRightPhysicalDescription,
            moduleControlConfig,
            moduleLimitConfig
        )

        private val frontLeft: Config = Config(
            frontLeftIdConfig,
            frontLeftStateConfig,
            frontLeftPhysicalDescription,
            moduleControlConfig,
            moduleLimitConfig
        )

        private val backLeft: Config = Config(
            backLeftIdConfig,
            backLeftStateConfig,
            backLeftPhysicalDescription,
            moduleControlConfig,
            moduleLimitConfig
        )

        private val backRight: Config = Config(
            backRightIdConfig,
            backRightStateConfig,
            backRightPhysicalDescription,
            moduleControlConfig,
            moduleLimitConfig
        )

        val moduleConfigurations: Array<Config> = arrayOf(frontRight, frontLeft, backRight, backLeft)
    }

    private val driveIdentifierConfig = SwerveDrive.IdentifierConfig(imuId)
    private val driveModuleConfig = SwerveDrive.ModuleConfig(moduleConfigurations)
    val swerveDriveConfig: SwerveDrive.Config = SwerveDrive.Config(driveModuleConfig, driveIdentifierConfig)
}
