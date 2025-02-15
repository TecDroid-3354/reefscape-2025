package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import net.tecdroid.conventions.RotationDirection;
import net.tecdroid.util.*;
import net.tecdroid.util.geometry.Rectangle;
import net.tecdroid.util.geometry.Square;
import net.tecdroid.util.geometry.Wheel;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.ControlConstants.*;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.Ids.*;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.ModuleState.*;
import static net.tecdroid.util.IdentifiersKt.joinDigits;

public class SwerveDriveConstants {

    static final int GYRO_ID = 1;

    // Drivetrain Measurements //
    static final Rectangle DRIVETRAIN_DIMENSIONS = new Square(Inches.of(22.23));

    // Module Characteristics //
    static final Wheel WHEEL = Wheel.Companion.fromRadius(Inches.of(2.0));

    static final GearRatio DRIVE_GEAR_RATIO = new GearRatio(6.12, 1.0);
    static final GearRatio STEER_GEAR_RATIO = new GearRatio(150.0, 7.0);

    // Limits //
    static final Current DRIVE_MOTOR_CURRENT_LIMIT = Amps.of(40.0);
    static final Current STEER_MOTOR_CURRENT_LIMIT = Amps.of(30.0);

    static final Time DRIVE_MOTOR_RAMP_RATE = Seconds.of(0.1);

    static final class Ids {
        static final DigitId FRONT_RIGHT_MODULE = new DigitId(1);
        static final DigitId FRONT_LEFT_MODULE  = new DigitId(2);
        static final DigitId BACK_LEFT_MODULE   = new DigitId(3);
        static final DigitId BACK_RIGHT_MODULE  = new DigitId(4);

        static final DigitId DRIVE_CONTROLLER_DIGIT = new DigitId(1);
        static final DigitId STEER_CONTROLLER_DIGIT = new DigitId(2);
        static final DigitId ABSOLUTE_ENCODER_DIGIT = new DigitId(3);

        static final CanId FRONT_RIGHT_DRIVE_CONTROLLER = joinDigits(FRONT_RIGHT_MODULE, DRIVE_CONTROLLER_DIGIT).toCanId();
        static final CanId FRONT_LEFT_DRIVE_CONTROLLER = joinDigits(FRONT_LEFT_MODULE, DRIVE_CONTROLLER_DIGIT).toCanId();
        static final CanId BACK_LEFT_DRIVE_CONTROLLER  = joinDigits(BACK_LEFT_MODULE, DRIVE_CONTROLLER_DIGIT).toCanId();
        static final CanId BACK_RIGHT_DRIVE_CONTROLLER = joinDigits(BACK_RIGHT_MODULE, DRIVE_CONTROLLER_DIGIT).toCanId();

        static final CanId FRONT_RIGHT_STEER_CONTROLLER = joinDigits(FRONT_RIGHT_MODULE, STEER_CONTROLLER_DIGIT).toCanId();
        static final CanId FRONT_LEFT_STEER_CONTROLLER = joinDigits(FRONT_LEFT_MODULE, STEER_CONTROLLER_DIGIT).toCanId();
        static final CanId BACK_LEFT_STEER_CONTROLLER  = joinDigits(BACK_LEFT_MODULE, STEER_CONTROLLER_DIGIT).toCanId();
        static final CanId BACK_RIGHT_STEER_CONTROLLER = joinDigits(BACK_RIGHT_MODULE, STEER_CONTROLLER_DIGIT).toCanId();

        static final CanId FRONT_RIGHT_ABSOLUTE_ENCODER = joinDigits(FRONT_RIGHT_MODULE, ABSOLUTE_ENCODER_DIGIT).toCanId();
        static final CanId FRONT_LEFT_ABSOLUTE_ENCODER  = joinDigits(FRONT_LEFT_MODULE, ABSOLUTE_ENCODER_DIGIT).toCanId();
        static final CanId BACK_LEFT_ABSOLUTE_ENCODER   = joinDigits(BACK_LEFT_MODULE, ABSOLUTE_ENCODER_DIGIT).toCanId();
        static final CanId BACK_RIGHT_ABSOLUTE_ENCODER  = joinDigits(BACK_RIGHT_MODULE, ABSOLUTE_ENCODER_DIGIT).toCanId();
    }

    static final class ModuleState {
        static final Angle FRONT_RIGHT_MAGNET_OFFSET = Rotations.of(-0.09130859375);
        static final Angle FRONT_LEFT_MAGNET_OFFSET  = Rotations.of(-0.38982578125);
        static final Angle BACK_LEFT_MAGNET_OFFSET   = Rotations.of(-0.345458984375);
        static final Angle BACK_RIGHT_MAGNET_OFFSET  = Rotations.of(+0.138427734375);

        static final Translation2d FRONT_RIGHT_MODULE_OFFSET = new Translation2d(DRIVETRAIN_DIMENSIONS.getWidth().div(+2), DRIVETRAIN_DIMENSIONS.getHeight().div(-2)); // FR |      2 ← 1
        static final Translation2d FRONT_LEFT_MODULE_OFFSET  = new Translation2d(DRIVETRAIN_DIMENSIONS.getWidth().div(+2), DRIVETRAIN_DIMENSIONS.getHeight().div(+2)); // FL |      ↓   ↑
        static final Translation2d BACK_LEFT_MODULE_OFFSET   = new Translation2d(DRIVETRAIN_DIMENSIONS.getWidth().div(-2), DRIVETRAIN_DIMENSIONS.getHeight().div(+2)); // BL |      3 → 4
        static final Translation2d BACK_RIGHT_MODULE_OFFSET  = new Translation2d(DRIVETRAIN_DIMENSIONS.getWidth().div(-2), DRIVETRAIN_DIMENSIONS.getHeight().div(-2)); // BR | Quadrant Convention
    }

    static final class ControlConstants {
        static final PidfCoefficients DRIVE_PIDF = new PidfCoefficients(0.001 * 0.0, 0.0, 0.00, 0.0);
        static final SvagGains        DRIVE_SVAG = new SvagGains(0.132, 0.12, 0.01, 0.0);

        static final PidfCoefficients STEER_PIDF = new PidfCoefficients(0.01, 0.0, 0.001, 0.0);
        static final SvagGains        STEER_SVAG = new SvagGains(0.0, 0.0, 0.0, 0.0);
    }

    static final class ModuleConfig {
        static final SwerveModule.IdentifierConfig FRONT_RIGHT_ID_CONFIG = new SwerveModule.IdentifierConfig(FRONT_RIGHT_DRIVE_CONTROLLER, FRONT_RIGHT_STEER_CONTROLLER, FRONT_RIGHT_ABSOLUTE_ENCODER);
        static final SwerveModule.IdentifierConfig FRONT_LEFT_ID_CONFIG = new SwerveModule.IdentifierConfig(FRONT_LEFT_DRIVE_CONTROLLER, FRONT_LEFT_STEER_CONTROLLER, FRONT_LEFT_ABSOLUTE_ENCODER);
        static final SwerveModule.IdentifierConfig BACK_LEFT_ID_CONFIG = new SwerveModule.IdentifierConfig(BACK_LEFT_DRIVE_CONTROLLER, BACK_LEFT_STEER_CONTROLLER, BACK_LEFT_ABSOLUTE_ENCODER);
        static final SwerveModule.IdentifierConfig BACK_RIGHT_ID_CONFIG = new SwerveModule.IdentifierConfig(BACK_RIGHT_DRIVE_CONTROLLER, BACK_RIGHT_STEER_CONTROLLER, BACK_RIGHT_ABSOLUTE_ENCODER);

        static final SwerveModule.StateConfig FRONT_RIGHT_STATE_CONFIG = new SwerveModule.StateConfig(FRONT_RIGHT_MAGNET_OFFSET, RotationDirection.CounterclockwisePositive, RotationDirection.CounterclockwisePositive);
        static final SwerveModule.StateConfig FRONT_LEFT_STATE_CONFIG  = new SwerveModule.StateConfig(FRONT_LEFT_MAGNET_OFFSET, RotationDirection.CounterclockwisePositive, RotationDirection.CounterclockwisePositive);
        static final SwerveModule.StateConfig BACK_LEFT_STATE_CONFIG   = new SwerveModule.StateConfig(BACK_LEFT_MAGNET_OFFSET, RotationDirection.CounterclockwisePositive, RotationDirection.CounterclockwisePositive);
        static final SwerveModule.StateConfig BACK_RIGHT_STATE_CONFIG  = new SwerveModule.StateConfig(BACK_RIGHT_MAGNET_OFFSET, RotationDirection.CounterclockwisePositive, RotationDirection.CounterclockwisePositive);

        static final SwerveModule.PhysicalDescription FRONT_RIGHT_PHYSICAL_DESCRIPTION = new SwerveModule.PhysicalDescription(FRONT_RIGHT_MODULE_OFFSET, DRIVE_GEAR_RATIO, STEER_GEAR_RATIO, WHEEL);
        static final SwerveModule.PhysicalDescription FRONT_LEFT_PHYSICAL_DESCRIPTION  = new SwerveModule.PhysicalDescription(FRONT_LEFT_MODULE_OFFSET, DRIVE_GEAR_RATIO, STEER_GEAR_RATIO, WHEEL);
        static final SwerveModule.PhysicalDescription BACK_LEFT_PHYSICAL_DESCRIPTION   = new SwerveModule.PhysicalDescription(BACK_LEFT_MODULE_OFFSET, DRIVE_GEAR_RATIO, STEER_GEAR_RATIO, WHEEL);
        static final SwerveModule.PhysicalDescription BACK_RIGHT_PHYSICAL_DESCRIPTION  = new SwerveModule.PhysicalDescription(BACK_RIGHT_MODULE_OFFSET, DRIVE_GEAR_RATIO, STEER_GEAR_RATIO, WHEEL);

        static final SwerveModule.ControlConfig MODULE_CONTROL_CONFIG = new SwerveModule.ControlConfig(DRIVE_PIDF, DRIVE_SVAG, STEER_PIDF, STEER_SVAG);
        static final SwerveModule.LimitConfig MODULE_LIMIT_CONFIG = new SwerveModule.LimitConfig(DRIVE_MOTOR_CURRENT_LIMIT, DRIVE_MOTOR_RAMP_RATE, STEER_MOTOR_CURRENT_LIMIT);

        static final SwerveModule.Config FRONT_RIGHT = new SwerveModule.Config(FRONT_RIGHT_ID_CONFIG, FRONT_RIGHT_STATE_CONFIG, FRONT_RIGHT_PHYSICAL_DESCRIPTION, MODULE_CONTROL_CONFIG, MODULE_LIMIT_CONFIG);
        static final SwerveModule.Config FRONT_LEFT  = new SwerveModule.Config(FRONT_LEFT_ID_CONFIG, FRONT_LEFT_STATE_CONFIG, FRONT_LEFT_PHYSICAL_DESCRIPTION, MODULE_CONTROL_CONFIG, MODULE_LIMIT_CONFIG);
        static final SwerveModule.Config BACK_LEFT   = new SwerveModule.Config(BACK_LEFT_ID_CONFIG, BACK_LEFT_STATE_CONFIG, BACK_LEFT_PHYSICAL_DESCRIPTION, MODULE_CONTROL_CONFIG, MODULE_LIMIT_CONFIG);
        static final SwerveModule.Config BACK_RIGHT  = new SwerveModule.Config(BACK_RIGHT_ID_CONFIG, BACK_RIGHT_STATE_CONFIG, BACK_RIGHT_PHYSICAL_DESCRIPTION, MODULE_CONTROL_CONFIG, MODULE_LIMIT_CONFIG);

        static final SwerveModule.Config[] CONFIGURATIONS = { FRONT_RIGHT, FRONT_LEFT, BACK_RIGHT, BACK_LEFT };
    }

    public static final SwerveDrive.Config CONFIG = new SwerveDrive.Config(ModuleConfig.CONFIGURATIONS, GYRO_ID);

}
