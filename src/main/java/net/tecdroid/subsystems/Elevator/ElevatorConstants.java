package net.tecdroid.subsystems.Elevator;

public final class ElevatorConstants {
    public static class ElevatorMotors {
        // Ids
        public static final int LEFT_MOTOR_ID = 0;
        public static final int RIGHT_MOTOR_ID = 1;

        // Limits
        public static final double AMP_LIMITS = 40.0;
        // Set inverted
        public static final boolean IS_INVERTED = true;

    }

    public static class AbsoluteEncoder {
        // Channel
        public static final int ABSOLUTE_ENCODER_CHANNEL = 2;

        // Set inverted
        public static final boolean IS_INVERTED = false;

        // Encoder zero position
        public static final double OFFSET_POSITION = 0.0;
    }

    public static class ElevatorReductions {
        // Gear Ratio
        public static final double ELEVATOR_GEAR_RATIO = 8.9285;
        public static final double ELEVATOR_INCHES_PER_REV = 2.246;
        public static final double ELEVATOR_GEAR_RATIO_CONVERSION_FACTOR = 1 / 8.9285;

    }

    public static class ElevatorCoefficients {
        public static final double G = 0.0;
        public static final double S = 0.25; // Add 0.25 V output to overcome static friction
        public static final double V = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double A = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double P = 4.8; // A position error of 2.5 rotations results in 12 V output
        public static final double I = 0.0; // no output for integrated error
        public static final double D = 0.1; // A velocity error of 1 rps results in 0.1 V output
    }

    public static class ElevatorMotionMagicSettings {
        public static final double MOTION_MAGIC_CRUISE_VELOCITY = 80.0; // Target cruise velocity of 80 rps
        public static final double MOTION_MAGIC_ACCELERATION = 160.0; // Target acceleration of 160 rps/s (0.5 seconds)
        public static final double MOTION_MAGIC_JERK = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    }
}
