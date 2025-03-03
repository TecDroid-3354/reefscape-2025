package net.tecdroid.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.util.RotationalDirection.Clockwise;
import static net.tecdroid.util.RotationalDirection.Counterclockwise;

import edu.wpi.first.units.measure.*;
import net.tecdroid.subsystems.intake.IntakeController.*;
import net.tecdroid.util.*;
import net.tecdroid.util.geometry.Wheel;

public class IntakeConfiguration {
    private static class DeviceIDs {
        private final DigitId intakeDigit = new DigitId(8);
        private final DigitId algaeIntakeDigit = new DigitId(1);
        private final DigitId coralIntakeDigit = new DigitId(2);
        final DeviceIdentifiers intakeDevicesIDs = new DeviceIdentifiers(
                IdentifiersKt.joinDigits(intakeDigit, algaeIntakeDigit),
                IdentifiersKt.joinDigits(intakeDigit, coralIntakeDigit)
        );
    }
    
    private static class Devices {
        private final MotorProperties algaeIntakeMotorProperties = Motors.INSTANCE.getKrakenX60();
        private final MotorProperties coralIntakeMotorProperties = Motors.INSTANCE.getKrakenX60();
        final DeviceProperties intakeDevicesProperties = new DeviceProperties(
                algaeIntakeMotorProperties, coralIntakeMotorProperties);
    }
    
    private static class Limits {
        private final Current algaeIntakeMotorCurrentLimit = Amps.of(40.0);
        private final Current coralIntakeMotorCurrentLimit = Amps.of(40.0);
        final DeviceLimits intakeDevicesLimits = new DeviceLimits(
                algaeIntakeMotorCurrentLimit, coralIntakeMotorCurrentLimit);
    }
    
    private static class Conventions {
        private final RotationalDirection algaeIntakeRotationalPositiveDirection = Clockwise;
        private final RotationalDirection coralIntakeRotationalPositiveDirection = Counterclockwise;
        final DeviceConventions intakeDevicesConventions = new DeviceConventions(
                algaeIntakeRotationalPositiveDirection, coralIntakeRotationalPositiveDirection
        );
    }
    
    private static class Structure {
        private final GearRatio algaeIntakeMotorGR = new GearRatio(6.75, 1, 0); // TODO: GEAR COUNT
        private final Wheel algaeIntakeWheel = Wheel.Companion.fromDiameter(Inches.of(4.0));
        private final GearRatio coralIntakeMotorGR = new GearRatio(4.5, 1, 7);
        private final Wheel coralIntakeWheel = Wheel.Companion.fromDiameter(Inches.of(4.0));
        final PhysicalDescription intakePhysicalDescription = new PhysicalDescription(
                algaeIntakeMotorGR, coralIntakeMotorGR,
                algaeIntakeWheel, coralIntakeWheel
        );
    }
    
    private static class Control {
        private final LinearVelocity algaeIntakeMaxWheelLinearVelocity = MetersPerSecond.of(2.0);
        private final LinearVelocity coralIntakeMaxWheelLinearVelocity = MetersPerSecond.of(2.0);
        private final Voltage retainAlgaeMinimumVoltage = Volts.of(0.18);
        private final Time intakeRampRate = Seconds.of(0.1);
        final ControlConstants intakeControlConstants = new ControlConstants(
                algaeIntakeMaxWheelLinearVelocity, coralIntakeMaxWheelLinearVelocity,
                retainAlgaeMinimumVoltage, intakeRampRate
        );
    }
    
    final Config intakeConfig = new Config(
            new DeviceIDs().intakeDevicesIDs, new Devices().intakeDevicesProperties, new Limits().intakeDevicesLimits,
            new Conventions().intakeDevicesConventions, new Structure().intakePhysicalDescription, new Control().intakeControlConstants
    );
}