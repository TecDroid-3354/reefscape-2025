package net.tecdroid.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.util.RotationalDirection.Clockwise;
import static net.tecdroid.util.RotationalDirection.Counterclockwise;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import net.tecdroid.util.*;

public class ClimberConfiguration {
    public static class ClimberIdentifiers {
        private static final NumericId leftClimberMotorID = new NumericId(1);
        private static final NumericId rightClimberMotorID = new NumericId(2);
        private static final NumericId throughboreID = new NumericId(3);

        public final ClimberController.DeviceIdentifiers deviceIdentifiers =
                new ClimberController.DeviceIdentifiers(leftClimberMotorID, rightClimberMotorID, throughboreID);
    }

    private static class Devices {
        private final MotorProperties leftclimberMotorProperties = Motors.INSTANCE.getKrakenX60();
        private final MotorProperties rightClimberMotorProperties = Motors.INSTANCE.getKrakenX60();
        final ClimberController.DeviceProperties climberDevicesProperties = new ClimberController.DeviceProperties(
            leftclimberMotorProperties, rightClimberMotorProperties);

    } //falta poner los amps
    private static class Limits {
        private final Current leftClimberMotorCurrentLimit = Amps.of(40);
        private final Current rightClimberMotorCurrentLimit = Amps.of(40);
        final ClimberController.DeviceLimits climberDeviceLimits = new ClimberController.DeviceLimits (
            leftClimberMotorCurrentLimit, rightClimberMotorCurrentLimit);
    }

    private static class Conventions {
        private final RotationalDirection leftClimberPositiveRotation = Clockwise;
        private final RotationalDirection rightClimberPositiveRotation = Counterclockwise;
        final ClimberController.DeviceConventions climberDiviceConections = new ClimberController.DeviceConventions (
            leftClimberPositiveRotation, rightClimberPositiveRotation);
        public ClimberController.DeviceConventions climberDevicesConventions;
    }

    private static class Structure{
        private final GearRatio climberMotorsGR = new GearRatio(288, 1, 0);
        final ClimberController.PhysicalDescription climberPhysicalDescription = new ClimberController.PhysicalDescription(
            climberMotorsGR);
    }
    //hay que revisar los MetersPerSecond
    private static class Control {
        private final Time climberRampRate = Seconds.of(1.0);
        private final PidfCoefficients climberPidfCoefficients = new PidfCoefficients(
                0.0,
                0.0,
                0.0,
                0.0
        );

        final ClimberController.ControlConstants climberControlConstants = new ClimberController.ControlConstants(
                climberRampRate, climberPidfCoefficients);
    }

    final ClimberController.Config intakeConfig = new ClimberController.Config(
            new ClimberController.DeviceIdentifiers(
                    ClimberIdentifiers.leftClimberMotorID,
                    ClimberIdentifiers.rightClimberMotorID,
                    ClimberIdentifiers.throughboreID),
            new Devices().climberDevicesProperties,
            new Limits().climberDeviceLimits,
            new Conventions().climberDevicesConventions,
            new Structure().climberPhysicalDescription,
            new Control().climberControlConstants
    );
}