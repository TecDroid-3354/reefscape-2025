package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import net.tecdroid.util.GearRatio;
import net.tecdroid.util.PidfConstants;
import net.tecdroid.util.SvaConstants;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.PI;
import static net.tecdroid.constants.UnitConstants.FULL_ROTATION;
import static net.tecdroid.util.MotorConstants.KRAKEN_MAX_RPM;

public class SwerveDriveConstants {

    static final int GYRO_ID = 1;

    // Measurements
    static final Distance TRACK_WIDTH            = Inches.of(22.23);
    static final Distance WHEEL_BASE             = Inches.of(22.23);

    // Obtained empirically through speedometer
    static final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(723.6);

    // Module
    static final Distance WHEEL_DIAMETER      = Inches.of(4);
    static final Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(PI);

    static final GearRatio DRIVE_GR = new GearRatio(6.12, 1.0);
    static final GearRatio STEER_GR = new GearRatio(150.0, 7.0);

    static final Distance           DRIVE_PCF = WHEEL_CIRCUMFERENCE.div(DRIVE_GR.getRatio());
    static final LinearVelocity     DRIVE_VCF = DRIVE_PCF.per(Minute);
    static final LinearAcceleration DRIVE_ACF = DRIVE_VCF.per(Second);

    static final Angle           STEER_PCF = FULL_ROTATION.div(STEER_GR.getRatio());
    static final AngularVelocity STEER_VCF = STEER_PCF.per(Minute);

    static final double DRIVE_MAX_POWER_FACTOR = 0.9;
    static final AngularVelocity DRIVE_MAX_ANGULAR_VELOCITY = KRAKEN_MAX_RPM.times(DRIVE_MAX_POWER_FACTOR);
    static final LinearVelocity DRIVE_MAX_LINEAR_VELOCITY = WHEEL_CIRCUMFERENCE.times(DRIVE_GR.apply(KRAKEN_MAX_RPM.in(RotationsPerSecond))).per(Second);

    static final Current DRIVE_CR = Amps.of(40.0);
    static final Current STEER_CR = Amps.of(30.0);

    static final Time DRIVE_RR = Seconds.of(0.1);
    static final Time STEER_RR = Seconds.of(0.1);

    static final class ModuleConfig {
        static final int ABS_ENCODER_ID_OFFSET      = 3;
        static final int DRIVE_CONTROLLER_ID_OFFSET = 1;
        static final int STEER_CONTROLLER_ID_OFFSET = 2;

        static final int FRONT_RIGHT_MODULE = 10;
        static final int FRONT_LEFT_MODULE  = 20;
        static final int BACK_LEFT_MODULE   = 30;
        static final int BACK_RIGHT_MODULE  = 40;

        static final int FRONT_RIGHT_DRIVE = FRONT_RIGHT_MODULE + DRIVE_CONTROLLER_ID_OFFSET;
        static final int FRONT_LEFT_DRIVE  = FRONT_LEFT_MODULE + DRIVE_CONTROLLER_ID_OFFSET;
        static final int BACK_LEFT_DRIVE   = BACK_LEFT_MODULE + DRIVE_CONTROLLER_ID_OFFSET;
        static final int BACK_RIGHT_DRIVE  = BACK_RIGHT_MODULE + DRIVE_CONTROLLER_ID_OFFSET;

        static final int FRONT_RIGHT_STEER = FRONT_RIGHT_MODULE + STEER_CONTROLLER_ID_OFFSET;
        static final int FRONT_LEFT_STEER  = FRONT_LEFT_MODULE + STEER_CONTROLLER_ID_OFFSET;
        static final int BACK_LEFT_STEER   = BACK_LEFT_MODULE + STEER_CONTROLLER_ID_OFFSET;
        static final int BACK_RIGHT_STEER  = BACK_RIGHT_MODULE + STEER_CONTROLLER_ID_OFFSET;

        static final int FRONT_RIGHT_ABSOLUTE_ENCODER = FRONT_RIGHT_MODULE + ABS_ENCODER_ID_OFFSET;
        static final int FRONT_LEFT_ABSOLUTE_ENCODER  = FRONT_LEFT_MODULE + ABS_ENCODER_ID_OFFSET;
        static final int BACK_LEFT_ABSOLUTE_ENCODER   = BACK_LEFT_MODULE + ABS_ENCODER_ID_OFFSET;
        static final int BACK_RIGHT_ABSOLUTE_ENCODER  = BACK_RIGHT_MODULE + ABS_ENCODER_ID_OFFSET;

        static final double FRONT_RIGHT_MAGNET_OFFSET = -0.09130859375;
        static final double FRONT_LEFT_MAGNET_OFFSET  = -0.38982578125;
        static final double BACK_LEFT_MAGNET_OFFSET   = -0.345458984375;
        static final double BACK_RIGHT_MAGNET_OFFSET  = +0.138427734375;

        static final Translation2d FRONT_RIGHT_MODULE_OFFSET = new Translation2d(TRACK_WIDTH.div(+2), WHEEL_BASE.div(-2)); // FR |      2 ← 1
        static final Translation2d FRONT_LEFT_MODULE_OFFSET  = new Translation2d(TRACK_WIDTH.div(+2), WHEEL_BASE.div(+2)); // FL |      ↓   ↑
        static final Translation2d BACK_LEFT_MODULE_OFFSET   = new Translation2d(TRACK_WIDTH.div(-2), WHEEL_BASE.div(-2)); // BL |      3 → 4
        static final Translation2d BACK_RIGHT_MODULE_OFFSET  = new Translation2d(TRACK_WIDTH.div(-2), WHEEL_BASE.div(+2)); // BR | Quadrant Convention

        static final SwerveModule.Config FRONT_RIGHT = new SwerveModule.Config(FRONT_RIGHT_MODULE_OFFSET, FRONT_RIGHT_DRIVE, FRONT_RIGHT_STEER, FRONT_RIGHT_ABSOLUTE_ENCODER, FRONT_RIGHT_MAGNET_OFFSET, false); // false
        static final SwerveModule.Config FRONT_LEFT  = new SwerveModule.Config(FRONT_LEFT_MODULE_OFFSET, FRONT_LEFT_DRIVE, FRONT_LEFT_STEER, FRONT_LEFT_ABSOLUTE_ENCODER, FRONT_LEFT_MAGNET_OFFSET, false); // true
        static final SwerveModule.Config BACK_LEFT   = new SwerveModule.Config(BACK_LEFT_MODULE_OFFSET, BACK_LEFT_DRIVE, BACK_LEFT_STEER, BACK_LEFT_ABSOLUTE_ENCODER, BACK_LEFT_MAGNET_OFFSET, false); // true
        static final SwerveModule.Config BACK_RIGHT  = new SwerveModule.Config(BACK_RIGHT_MODULE_OFFSET, BACK_RIGHT_DRIVE, BACK_RIGHT_STEER, BACK_RIGHT_ABSOLUTE_ENCODER, BACK_RIGHT_MAGNET_OFFSET, false); // false

        static final SwerveModule.Config[] CONFIGURATIONS = { FRONT_RIGHT, FRONT_LEFT, BACK_RIGHT, BACK_LEFT };
    }

    static final class Pidf {
        static final PidfConstants DRIVE = new PidfConstants(0.001 * 0.0, 0.0, 0.00, 0.0);
        static final PidfConstants STEER = new PidfConstants(0.005, 0.0, 0.002, 0.0);
        static final PidfConstants ANGLE = new PidfConstants(0.0055, 0.0, 0.002, 0.0);
        static final PidfConstants ALIGN = new PidfConstants(0.005, 0.0, 0.002, 0.0);
    }

    static final class Sva {
        static final SvaConstants DRIVE = new SvaConstants(0.132, 0.12, 0.01);
        // V = ks + v * kV
        // 12 = 0.15 +
    }

    public static final SwerveDrive.Config CONFIG = new SwerveDrive.Config(ModuleConfig.CONFIGURATIONS, GYRO_ID);

}
