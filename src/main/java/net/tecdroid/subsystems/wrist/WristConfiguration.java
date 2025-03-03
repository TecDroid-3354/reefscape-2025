package net.tecdroid.subsystems.wrist;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.subsystems.wrist.WristController.*;
import static net.tecdroid.util.RotationalDirection.Clockwise;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import net.tecdroid.util.*;


public class WristConfiguration {

    private static class DeviceIDs {
        private final DigitId wristDigit = new DigitId(7);
        private final DigitId throughborePort = new DigitId(0);
        private final DigitId leftMotorDigit = new DigitId(1);
        private final DigitId rightMotorDigit = new DigitId(2);

        final DeviceIdentifiers wristDevicesIDs = new DeviceIdentifiers(
                throughborePort,
                IdentifiersKt.joinDigits(wristDigit, leftMotorDigit),
                IdentifiersKt.joinDigits(wristDigit, rightMotorDigit)
        );
    }

    private static class Devices {
        private final MotorProperties wristMotorProperties = Motors.INSTANCE.getNeo();
        final DeviceProperties wristDeviceProperties = new DeviceProperties(wristMotorProperties);
    }

    private static class Limits {
        private final Current motorsCurrentLimit = Amps.of(30.0);
        private final Angle minimumAngleLimit = Degrees.of(-30.0); // TODO: GET REAL MINIMUM. ARBITRARY VALUE
        private final Angle maximumAngleLimit = Degrees.of(45.0); // TODO: GET REAL MAXIMUM. ARBITRARY VALUE
        private final AngularVelocity maximumAngularVelocity = RotationsPerSecond.of(31.53); // A third of motor's capacity
        final DeviceLimits wristDeviceLimits = new DeviceLimits(
                motorsCurrentLimit, minimumAngleLimit, maximumAngleLimit, maximumAngularVelocity);
    }

    private static class Conventions {
        private final RotationalDirection wristPositiveDirection = Clockwise;
        final DeviceConventions wristDeviceConventions = new DeviceConventions(wristPositiveDirection);
    }

    private static class Structure {
        private final GearRatio wristGR = new GearRatio(89.2857, 1.0, 0);
        private final Angle encoderOffset = Degrees.of(45.0); // TODO: GET REAL OFFSET. ARBITRARY VALUE
        final PhysicalDescription wristPhysicalDescription = new PhysicalDescription(wristGR, encoderOffset);
    }

    private static class Control {
        // TODO: Determine all PIDF, SVAG coefficients
        private final PidfCoefficients motorsPIDF = new PidfCoefficients(0.0, 0.0, 0.0, 0.0);
        private final SvagGains motorsSVAG = new SvagGains(0.0, 0.0, 0.0, 0.0);
        private final Time motorsRampRate = Seconds.of(0.1);
        final ControlConstants wristControlConstants = new ControlConstants(motorsPIDF, motorsSVAG, motorsRampRate);
    }

    final Config wristConfig = new Config(
            new DeviceIDs().wristDevicesIDs, new Devices().wristDeviceProperties, new Limits().wristDeviceLimits,
            new Conventions().wristDeviceConventions, new Structure().wristPhysicalDescription, new Control().wristControlConstants
    );
}
