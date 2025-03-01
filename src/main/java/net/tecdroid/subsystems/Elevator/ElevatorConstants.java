package net.tecdroid.subsystems.Elevator;

public final class ElevatorConstants {
    public static class Elevator {
        // Ids
        public static final int kLeftMotorID = 0;
        public static final int kRightMotorID = 1;

        // Set inverted
        public static final boolean kIsInverted = false;

    }

    public static class AbsoluteEncoder {
        // Channel
        public static final int kAbsoluteEncoderChannel = 2;

        // Set inverted
        public static final boolean kIsInverted = false;

        // Encoder zero position
        public static final double kOffsetPosition = 0.0;
    }

    public static class ElevatorReductions {
        // Gear Ratio
        public static final double elevatorGearRatio = 8.9285;
        public static final double elevatorGearRatioConversionFactor = 1 / 8.9285;

        public static final double elevatorInchesPerRev = 2.246;

    }

    public static class ElevatorCoefficients {
        public static final double kG = 0.0;
        public static final double kS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        public static final double kI = 0.0; // no output for integrated error
        public static final double kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    }
}
