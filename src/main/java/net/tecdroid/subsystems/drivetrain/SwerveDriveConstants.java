package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import net.tecdroid.util.GearRatio;
import net.tecdroid.util.PidfConstants;
import net.tecdroid.util.SvaConstants;
import net.tecdroid.util.UnitHelpers;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.PI;
import static net.tecdroid.constants.UnitConstants.FULL_ROTATION;

public class SwerveDriveConstants {

    private static final int GYRO_ID = 1;

    // Measurements
    private static final Distance TRACK_WIDTH            = Inches.of(22.23);
    private static final Distance WHEEL_BASE             = Inches.of(22.23);

    // Obtained empirically through speedometer
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(723.6);

    // Module
    private static final Distance WHEEL_DIAMETER      = Inches.of(4);
    private static final Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(PI);

    private static final GearRatio DRIVE_GR = new GearRatio(6.12, 1.0);
    private static final GearRatio STEER_GR = 150.0 / 7.0;

    public static final Distance       DRIVE_PCF = WHEEL_CIRCUMFERENCE.div(DRIVE_GR);
    public static final LinearVelocity DRIVE_VCF = DRIVE_PCF.per(Minute);

    public static final Angle           STEER_PCF = FULL_ROTATION.div(DRIVE_GR);
    public static final AngularVelocity STEER_VCF = STEER_PCF.per(Minute);

    public static final Current DRIVE_CR = Amps.of(40.0);
    public static final Current STEER_CR = Amps.of(30.0);

    public static final Time DRIVE_RR = Seconds.of(0.1);
    public static final Time STEER_RR = Seconds.of(0.1);

    public static final class ModuleConfig {
        private static final int ABS_ENCODER_ID_OFFSET      = 0;
        private static final int DRIVE_CONTROLLER_ID_OFFSET = 1;
        private static final int STEER_CONTROLLER_ID_OFFSET = 2;

        private static final int FRONT_RIGHT_MODULE = 10;
        private static final int FRONT_LEFT_MODULE  = 20;
        private static final int BACK_LEFT_MODULE   = 30;
        private static final int BACK_RIGHT_MODULE  = 40;

        private static final int FRONT_RIGHT_DRIVE = FRONT_RIGHT_MODULE + DRIVE_CONTROLLER_ID_OFFSET;
        private static final int FRONT_LEFT_DRIVE  = FRONT_LEFT_MODULE + DRIVE_CONTROLLER_ID_OFFSET;
        private static final int BACK_LEFT_DRIVE   = BACK_LEFT_MODULE + DRIVE_CONTROLLER_ID_OFFSET;
        private static final int BACK_RIGHT_DRIVE  = BACK_RIGHT_MODULE + DRIVE_CONTROLLER_ID_OFFSET;

        private static final int FRONT_RIGHT_STEER = FRONT_RIGHT_MODULE + STEER_CONTROLLER_ID_OFFSET;
        private static final int FRONT_LEFT_STEER  = FRONT_LEFT_MODULE + STEER_CONTROLLER_ID_OFFSET;
        private static final int BACK_LEFT_STEER   = BACK_LEFT_MODULE + STEER_CONTROLLER_ID_OFFSET;
        private static final int BACK_RIGHT_STEER  = BACK_RIGHT_MODULE + STEER_CONTROLLER_ID_OFFSET;

        private static final int FRONT_RIGHT_ABSOLUTE_ENCODER = FRONT_RIGHT_MODULE + ABS_ENCODER_ID_OFFSET;
        private static final int FRONT_LEFT_ABSOLUTE_ENCODER  = FRONT_LEFT_MODULE + ABS_ENCODER_ID_OFFSET;
        private static final int BACK_LEFT_ABSOLUTE_ENCODER   = BACK_LEFT_MODULE + ABS_ENCODER_ID_OFFSET;
        private static final int BACK_RIGHT_ABSOLUTE_ENCODER  = BACK_RIGHT_MODULE + ABS_ENCODER_ID_OFFSET;

        private static final double FRONT_RIGHT_MAGNET_OFFSET = -0.0966796875;
        private static final double FRONT_LEFT_MAGNET_OFFSET  = -0.396484375;
        private static final double BACK_LEFT_MAGNET_OFFSET   = -0.34033203125;
        private static final double BACK_RIGHT_MAGNET_OFFSET  = 0.13623046875;

        private static final Translation2d FRONT_RIGHT_MODULE_OFFSET = new Translation2d(TRACK_WIDTH.div(+2), WHEEL_BASE.div(-2)); // FR |      2 ← 1
        private static final Translation2d FRONT_LEFT_MODULE_OFFSET  = new Translation2d(TRACK_WIDTH.div(+2), WHEEL_BASE.div(+2)); // FL |      ↓   ↑
        private static final Translation2d BACK_LEFT_MODULE_OFFSET   = new Translation2d(TRACK_WIDTH.div(-2), WHEEL_BASE.div(-2)); // BL |      3 → 4
        private static final Translation2d BACK_RIGHT_MODULE_OFFSET  = new Translation2d(TRACK_WIDTH.div(-2), WHEEL_BASE.div(+2)); // BR | Quadrant Convention

        private static final SwerveModule.Config FRONT_RIGHT = new SwerveModule.Config(FRONT_RIGHT_MODULE_OFFSET, FRONT_RIGHT_DRIVE, FRONT_RIGHT_STEER, FRONT_RIGHT_ABSOLUTE_ENCODER, FRONT_RIGHT_MAGNET_OFFSET);
        private static final SwerveModule.Config FRONT_LEFT  = new SwerveModule.Config(FRONT_LEFT_MODULE_OFFSET, FRONT_LEFT_DRIVE, FRONT_LEFT_STEER, FRONT_LEFT_ABSOLUTE_ENCODER, FRONT_LEFT_MAGNET_OFFSET);
        private static final SwerveModule.Config BACK_LEFT   = new SwerveModule.Config(BACK_LEFT_MODULE_OFFSET, BACK_LEFT_DRIVE, BACK_LEFT_STEER, BACK_LEFT_ABSOLUTE_ENCODER, BACK_LEFT_MAGNET_OFFSET);
        private static final SwerveModule.Config BACK_RIGHT  = new SwerveModule.Config(BACK_RIGHT_MODULE_OFFSET, BACK_RIGHT_DRIVE, BACK_RIGHT_STEER, BACK_RIGHT_ABSOLUTE_ENCODER, BACK_RIGHT_MAGNET_OFFSET);

        public static final SwerveModule.Config[] CONFIGURATIONS = { FRONT_RIGHT, FRONT_LEFT, BACK_RIGHT, BACK_LEFT };
    }

    public static final class Pidf {
        public static final PidfConstants DRIVE = new PidfConstants(0.008, 0.0, 0.012, 0.2);
        public static final PidfConstants STEER = new PidfConstants(0.005, 0.0, 0.002, 0.0);
        public static final PidfConstants ANGLE = new PidfConstants(0.0055, 0.0, 0.002, 0.0);
        public static final PidfConstants ALIGN = new PidfConstants(0.005, 0.0, 0.002, 0.0);
    }

    public static final class Sva {
        public static final SvaConstants DRIVE = new SvaConstants(0.0, 0.0, 0.0);
    }

    public static final SwerveDrive.Config CONFIG = new SwerveDrive.Config(ModuleConfig.CONFIGURATIONS, GYRO_ID);

}
