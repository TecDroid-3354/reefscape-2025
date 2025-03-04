package net.tecdroid.subsystems.Elevator;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import net.tecdroid.subsystems.Elevator.Elevator.*;
import net.tecdroid.util.GearRatio;
import net.tecdroid.util.MotionMagicSettings;
import net.tecdroid.util.PidfCoefficients;
import net.tecdroid.util.SvagGains;

import static edu.wpi.first.units.Units.*;

public final class ElevatorConfig {
    private static class ElevatorIdentifiers {
        // Ids
        private static final int LEFT_MOTOR_ID = 0;
        private static final int RIGHT_MOTOR_ID = 1;

        // Channel
        public static final int ABSOLUTE_ENCODER_CHANNEL = 2;

        private final DeviceIdentifier deviceIdentifier = new DeviceIdentifier(LEFT_MOTOR_ID, RIGHT_MOTOR_ID, ABSOLUTE_ENCODER_CHANNEL);

    }

    private static class ElevatorMotorProperties {

        // Limits
        private static final Current AMP_LIMITS = Amps.of(40.0);

        // Set inverted
        private static final InvertedValue LEADER_MOTOR_INVERTED_TYPE = InvertedValue.Clockwise_Positive;
        private static final boolean FOLLOWER_MOTOR_INVERTED = true;

        private final MotorProperties motorProperties = new MotorProperties(AMP_LIMITS, LEADER_MOTOR_INVERTED_TYPE, FOLLOWER_MOTOR_INVERTED);

    }

    private static class AbsoluteEncoder {
        // Set inverted
        private static final boolean IS_INVERTED = false;

        // Encoder zero position
        private static final Angle OFFSET_POSITION = Rotations.of(0.0);

        private final EncoderProperties encoderProperties = new EncoderProperties(IS_INVERTED, OFFSET_POSITION);
    }

    private static class ElevatorReductions {
        // Gear Ratio
        public static final double ELEVATOR_GEAR_RATIO = 8.9285;
        public static final Distance ELEVATOR_INCHES_PER_REV = Distance.ofBaseUnits(2.246, Inches);

        private final GearRatio motorGearRatio = new GearRatio(ELEVATOR_GEAR_RATIO, 1, 0);

        private final ElevatorGearRatio elevatorGearRatio = new ElevatorGearRatio(motorGearRatio, ELEVATOR_INCHES_PER_REV);

    }

    private static class ElevatorCoefficients {
        public static final double G = 0.0;
        public static final double S = 0.25; // Add 0.25 V output to overcome static friction
        public static final double V = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double A = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double P = 4.8; // A position error of 2.5 rotations results in 12 V output
        public static final double I = 0.0; // no output for integrated error
        public static final double D = 0.1; // A velocity error of 1 rps results in 0.1 V output
        public static final double F = 0.0;

        private final Coefficients coefficients = new Coefficients(
                new PidfCoefficients(P, I, D, F), new SvagGains(S, V, A, G));
    }

    private static class ElevatorMotionMagicSettings {
        public static final double MOTION_MAGIC_CRUISE_VELOCITY = 80.0; // Target cruise velocity of 80 rps
        public static final double MOTION_MAGIC_ACCELERATION = 160.0; // Target acceleration of 160 rps/s (0.5 seconds)
        public static final double MOTION_MAGIC_JERK = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        final MotionMagicProperties motionMagicProperties = new MotionMagicProperties(
                new MotionMagicSettings(MOTION_MAGIC_CRUISE_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
        );
    }

    public static final Elevator.ElevatorConfig elevatorConfig = new Elevator.ElevatorConfig(
            new ElevatorIdentifiers().deviceIdentifier, new ElevatorMotorProperties().motorProperties,
            new ElevatorReductions().elevatorGearRatio, new AbsoluteEncoder().encoderProperties,
            new ElevatorCoefficients().coefficients, new ElevatorMotionMagicSettings().motionMagicProperties
    );

}
