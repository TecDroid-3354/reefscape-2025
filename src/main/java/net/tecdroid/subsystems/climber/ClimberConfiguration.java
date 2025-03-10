package net.tecdroid.subsystems.climber;

import edu.wpi.first.units.measure.*;
import net.tecdroid.util.*;
import edu.wpi.first.units.measure.Time;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import net.tecdroid.util.RotationalDirection;

public class ClimberConfiguration {
    public static class ClimberIdentifiers {
        private static final DigitId digitModule = new DigitId(5);

        // leading climber motor --> left motor
        private final DigitId leadingClimberMotorID = new DigitId(5);

        // follower climber motor --> right  motor
        private final DigitId followerClimberMotorID = new DigitId(6);

        // TODO: CHECK THE THROUGHBORE'S ACTUAL ID
        private final DigitId throughboreClimberPort = new DigitId(7); // TODO: CHECK THE THROUGHBORE'S ACTUAL ID

        final Climber.DeviceIdentifiers deviceIdentifiers = new Climber.DeviceIdentifiers(
                throughboreClimberPort,
                IdentifiersKt.joinDigits(digitModule, leadingClimberMotorID),
                IdentifiersKt.joinDigits(digitModule, followerClimberMotorID)
                );
    }

    private static class DeviceProperties {
        private final MotorProperties climberMotorProperties = Motors.INSTANCE.getKrakenX60();

        final Climber.DeviceProperties climberDevicesProperties =
                new Climber.DeviceProperties(
                        climberMotorProperties);
    }

    private static class Limits {
        private final Current climberMotorsCurrentLimit = Amps.of(40.0);

        // TODO: CHECK AND SET MINIMUM/MAXIMUM CLIMBER ANGLE DIRECTIONS
        // TODO: THESE ARE ONLY ARBITRARY VALUES
        private final Angle minimumClimberAngle = Degrees.of(0.0);
        private final Angle maximumClimberAngle = Degrees.of(90.0);

        final Climber.DeviceLimits climberDeviceLimits =
                new Climber.DeviceLimits(
                        climberMotorsCurrentLimit,
                        minimumClimberAngle,
                        maximumClimberAngle);
    }

    private static class Conventions {
        private final RotationalDirection climberRotationalPositiveDirection = RotationalDirection.Clockwise;

        final Climber.DeviceConventions climberDeviceConventions =
                new Climber.DeviceConventions (climberRotationalPositiveDirection);
    }

    private static class Structure {
        // Setting a common physical gear ratio for both motors
        private final GearRatio climberMotorsGR = new GearRatio(288, 1, 0);
        private final Angle encoderOffset = Degrees.of(45.0); // TODO: GET REAL VALUE | 45 IS AN ARBITRARY VALUE

        final Climber.PhysicalDescription climberPhysicalDescription =
                new Climber.PhysicalDescription(
                        climberMotorsGR, encoderOffset);
    }
    //hay que revisar los MetersPerSecond
    private static class Control {
        private final MotionTargets climberMotionTargets = new MotionTargets(0.0, 0.0, 0.0);
        private final Time climberRampRate = Seconds.of(0.1);
        private final NeutralModeValue climberNeutralModeValue = NeutralModeValue.Brake;

        final Climber.ControlConstants climberControlConstants = new Climber.ControlConstants(
                climberMotionTargets, climberRampRate, climberNeutralModeValue);
    }

    static final Climber.Config climberConfiguration = new Climber.Config(
            new ClimberIdentifiers().deviceIdentifiers,
            new DeviceProperties().climberDevicesProperties,
            new Limits().climberDeviceLimits,
            new Conventions().climberDeviceConventions,
            new Structure().climberPhysicalDescription,
            new Control().climberControlConstants
    );
}

