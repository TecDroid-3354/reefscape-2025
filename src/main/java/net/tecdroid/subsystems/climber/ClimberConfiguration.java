package net.tecdroid.subsystems.climber;

import net.tecdroid.util.*;

public class ClimberConfiguration {
    public static class ClimberIdentifiers {
        private final NumericId leftClimberMotorID = new NumericId(1);
        private final NumericId rightClimberMotorID = new NumericId(2);

        ClimberController.DeviceIdentifiers deviceIdentifiers = new ClimberController.DeviceIdentifiers(leftClimberMotorID,
                rightClimberMotorID);
    }

    private static class Devices {
        private final MotorProperties leftclimberMotorProperties = Motors.INSTANCE.getKrakenX60();
        private final MotorProperties rightClimberMotorProperties = Motors.INSTANCE.getKrakenX60();
        final DiviceProperties climberDevicesProperties = new DeviceProperties(
            leftclimberMotorProperties, rightClimberMotorProperties);

    }
    private static class Limits {
        private final Current leftClimberMotorCurrentLimit = Amps.of();
        private final Current rightClimberMotorCurrentLimit = Amps.of();
        final DiviceLimits climberDeviceLimits = new Devicelimits (
            leftClimberMotorCurrentLimit, rightClimberMotorCurrentLimit);
    }

    private static class Conventions {
        private final RationalDirection leftClimberPositiveRotation = Clockwise;
        private final RationalDirection rightClimberPositiveRotation = CounterClockwise;
        final DiviceConventions climberDiviceConections = new DiviceConventions (
            leftClimberPositiveRotation, rightClimberPositiveRotation);
    }

    private static class Structure{
        private final GearRatio climberMotorsGR = new GearRatio(288, 1, 0);
        final PhysicalDescription intakePhysicalDescription = new PhysicalDescription(
            climberMotorsGR);
    }
    
    private static class Control {
        private final LinearVelocity leftClimberMaxWheelLinearVelocity = MetersPerSecond.of(1.0);
        private final LinearVelocity rightClimberMaxWheelLinearVelocity = MetersPerSecond.of(1.0);
        final 
    }
}
