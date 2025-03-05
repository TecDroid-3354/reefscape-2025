package net.tecdroid.subsystems.climber;

import edu.wpi.first.units.measure.Current;
import net.tecdroid.util.*;

import static edu.wpi.first.units.Units.Amps;

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
        final ClimberController.DeviceProperties climberDevicesProperties = new ClimberController.DeviceProperties(
            leftclimberMotorProperties, rightClimberMotorProperties);

    } //falta poner los amps
    private static class Limits {
        private final Current leftClimberMotorCurrentLimit = Amps.of();
        private final Current rightClimberMotorCurrentLimit = Amps.of();
        final DeviceLimits climberDeviceLimits = new Deviceslimits (
            leftClimberMotorCurrentLimit, rightClimberMotorCurrentLimit);
    }

    private static class Conventions {
        private RationalDirection Clockwise;
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
    //hay que revisar los MetersPerSecond
    private static class Control {
        private final LinearVelocity leftClimberMaxWheelLinearVelocity = MetersPerSecond.of(1.0);
        private final LinearVelocity rightClimberMaxWheelLinearVelocity = MetersPerSecond.of(1.0);
        private final Voltage retainMotorsMinimumVoltage = Volts.of(0.18);
        private final Time climberRampRate = Seconds.of(0.1);
        final ControlConstants intakeControlConstants = new ControlConstants(
                leftClimberMaxWheelLinearVelocity, rightClimberMaxWheelLinearVelocity,
                retainMotorsMinimumVoltage, climberRampRate);
        public Object climberControlConstants;
    }

    final Config intakeConfig = new Config(
            new DeviceIDs().climberDevicesIDs, new Devices().climberDevicesProperties, new Limits().climberDevicesLimits,
            new Conventions().climberDevicesConventions, new Structure().climberPhysicalDescription, new Control().climberControlConstants
    );
}

