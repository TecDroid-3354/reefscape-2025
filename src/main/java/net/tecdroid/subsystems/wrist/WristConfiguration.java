package net.tecdroid.subsystems.wrist;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.subsystems.wrist.WristController.*;
import static net.tecdroid.util.RotationalDirection.Clockwise;
import static com.ctre.phoenix6.signals.NeutralModeValue.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import net.tecdroid.util.*;


public class WristConfiguration {

    private static class DeviceIDs {
        private final DigitId wristDigit = new DigitId(6);
        private final DigitId throughborePort = new DigitId(0);
        private final DigitId wristMotorDigit = new DigitId(1);
        final DeviceIdentifiers wristDevicesIDs = new DeviceIdentifiers(
                IdentifiersKt.joinDigits(wristDigit, wristMotorDigit),
                throughborePort
        );
    }

    private static class Devices {
        private final MotorProperties wristMotorProperties = Motors.INSTANCE.getKrakenX60();
        final DeviceProperties wristDeviceProperties = new DeviceProperties(wristMotorProperties);
    }

    private static class Limits {
        private final Current motorsCurrentLimit = Amps.of(30.0);
        private final Angle minimumAngleLimit = Degrees.of(-30.0); // TODO: GET REAL MINIMUM. ARBITRARY VALUE
        private final Angle maximumAngleLimit = Degrees.of(45.0); // TODO: GET REAL MAXIMUM. ARBITRARY VALUE
        private final AngularVelocity maximumAngularVelocity = RotationsPerSecond.of(33.3333); // A third of motor's capacity
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
        private final PidfCoefficients motorPIDF = new PidfCoefficients(0.0, 0.0, 0.0, 0.0);
        private final SvagGains motorSVAG = new SvagGains(0.0, 0.0, 0.0, 0.0);
        private final NeutralModeValue motorNeutralMode = Brake;
        private final Time motorsRampRate = Seconds.of(0.1);
        final ControlConstants wristControlConstants = new ControlConstants(
                motorPIDF, motorSVAG, motorNeutralMode, motorsRampRate);
    }

    final Config wristConfig = new Config(
            new DeviceIDs().wristDevicesIDs, new Devices().wristDeviceProperties, new Limits().wristDeviceLimits,
            new Conventions().wristDeviceConventions, new Structure().wristPhysicalDescription, new Control().wristControlConstants
    );
}
