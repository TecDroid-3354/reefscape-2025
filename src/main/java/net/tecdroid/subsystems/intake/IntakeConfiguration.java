package net.tecdroid.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.util.RotationalDirection.Clockwise;

import edu.wpi.first.units.measure.*;
import net.tecdroid.subsystems.intake.IntakeController.*;
import net.tecdroid.util.*;
import net.tecdroid.util.geometry.Wheel;

public class IntakeConfiguration {
    private static class DeviceIDs {
        private final DigitId intakeDigit = new DigitId(6);
        private final DigitId intakeMotorDigit = new DigitId(2);
        final DeviceIdentifiers intakeDevicesIDs = new DeviceIdentifiers(
                IdentifiersKt.joinDigits(intakeDigit, intakeMotorDigit)
        );
    }
    
    private static class Devices {
        private final MotorProperties intakeMotorProperties = Motors.INSTANCE.getKrakenX60();
        final DeviceProperties intakeDevicesProperties = new DeviceProperties(intakeMotorProperties);
    }
    
    private static class Limits {
        private final Current intakeMotorCurrentLimit = Amps.of(40.0);
        final DeviceLimits intakeDevicesLimits = new DeviceLimits(intakeMotorCurrentLimit);
    }
    
    private static class Conventions {
        private final RotationalDirection intakeRotationalPositiveDirection = Clockwise;
        final DeviceConventions intakeDevicesConventions = new DeviceConventions(
                intakeRotationalPositiveDirection
        );
    }
    
    private static class Structure {
        private final GearRatio intakeMotorGR = new GearRatio(6.75, 1, 0); // TODO: GEAR COUNT
        private final Wheel intakeWheel = Wheel.Companion.fromDiameter(Inches.of(2.0));
        final PhysicalDescription intakePhysicalDescription = new PhysicalDescription(
                intakeMotorGR, intakeWheel
        );
    }
    
    private static class Control {
        private final LinearVelocity intakeMaxWheelLinearVelocity = MetersPerSecond.of(2.0);
        private final Voltage retainAlgaeMinimumVoltage = Volts.of(0.18);
        private final Time intakeRampRate = Seconds.of(0.1);
        final ControlConstants intakeControlConstants = new ControlConstants(
                intakeMaxWheelLinearVelocity, retainAlgaeMinimumVoltage, intakeRampRate
        );
    }
    
    final Config intakeConfig = new Config(
            new DeviceIDs().intakeDevicesIDs, new Devices().intakeDevicesProperties, new Limits().intakeDevicesLimits,
            new Conventions().intakeDevicesConventions, new Structure().intakePhysicalDescription, new Control().intakeControlConstants
    );
}