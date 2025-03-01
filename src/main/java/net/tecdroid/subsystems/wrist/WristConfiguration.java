package net.tecdroid.subsystems.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static net.tecdroid.subsystems.wrist.WristController.*;
import static net.tecdroid.util.RotationalDirection.Clockwise;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import net.tecdroid.util.*;


public class WristConfiguration {

    private class DeviceIDs {
        private final DigitId wristID = new DigitId(7);
        private final DigitId throughboreDigit = new DigitId(0);
        private final DigitId leftMotorDigit = new DigitId(1);
        private final DigitId rightMotorDigit = new DigitId(2);

        final DeviceIdentifiers wristDevicesIDs = new DeviceIdentifiers(
                throughboreDigit,
                IdentifiersKt.joinDigits(wristID, leftMotorDigit),
                IdentifiersKt.joinDigits(wristID, rightMotorDigit)
        );
    }

    private class Devices {
        private final MotorProperties wristMotorProperties = Motors.INSTANCE.getNeo();
        final DeviceProperties deviceProperties = new DeviceProperties(wristMotorProperties);
    }

    private class Limits {
        private final Current motorsCurrentLimit = Amps.of(30.0);
        private final Angle minimumAngleLimit = Angle.ofBaseUnits(60.0, Degrees);
        private final Angle maximumAngleLimit = Angle.ofBaseUnits(120.0, Degrees);
        final DeviceLimits deviceLimits = new DeviceLimits(motorsCurrentLimit, minimumAngleLimit, maximumAngleLimit);
    }

    private class Conventions {
        private final RotationalDirection wristPositiveDirection = Clockwise;
        final DeviceConventions deviceConventions = new DeviceConventions(wristPositiveDirection);
    }

    private class Structure {
        private final GearRatio wristGR = new GearRatio(89.2857, 1.0, 0);
        private final Angle encoderOffset = Angle.ofBaseUnits(45.0, Degrees); // TODO: Offset varies depending on the robot's build. 45 is an arbitrary value
        final PhysicalDescription physicalDescription = new PhysicalDescription(wristGR, encoderOffset);
    }

    private class Control {
        // TODO: Determine all PIDF, SVAG coefficients
        private final PidfCoefficients motorsPIDF = new PidfCoefficients(0.0, 0.0, 0.0, 0.0);
        private final SvagGains motorsSVAG = new SvagGains(0.0, 0.0, 0.0, 0.0);
        final ControlConstants controlConstants = new ControlConstants(motorsPIDF, motorsSVAG);
    }

    Config wristConfig = new Config(
            new DeviceIDs().wristDevicesIDs, new Devices().deviceProperties, new Limits().deviceLimits,
            new Conventions().deviceConventions, new Structure().physicalDescription, new Control().controlConstants
    );
}
