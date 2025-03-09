package net.tecdroid.subsystems.elevatorjoint;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

import edu.wpi.first.units.measure.Time;
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint.*;
import net.tecdroid.util.*;

import static edu.wpi.first.units.Units.*;


public class ElevatorJointConfiguration {
    private static class DeviceIDS {
        private final DigitId mElevatorAngleDigit = new DigitId(6);
        private final DigitId mLeadingMotorDigit = new DigitId(1);
        private final DigitId mFollowerMotorDigit = new DigitId(2);
        private final DigitId TroughBorePort = new DigitId(3);

        final DeviceIdentifiers elevatorAngleDevicesIDs = new DeviceIdentifiers(
                IdentifiersKt.joinDigits(mElevatorAngleDigit, mLeadingMotorDigit),
                IdentifiersKt.joinDigits(mElevatorAngleDigit, mFollowerMotorDigit),
                TroughBorePort
        );
    }

    private static class Devices {
        private final MotorProperties elevatorAngleMotorProperties = Motors.INSTANCE.getKrakenX60();

        final DeviceProperties elevatorAngleDeviceProperties = new DeviceProperties(
                elevatorAngleMotorProperties
        );
    }

    private static class Limits {
        private final Current motorsCurrent = Amps.of(40);
        private final Angle minimumElevatorAngle = Degrees.of(10.0);
        private final Angle maximumElevatorAngle = Degrees.of(90.0);
        private final Angle gravityPointAngle = Degrees.of(75.0);

        final DeviceLimits elevatorAngleDeviceLimits = new DeviceLimits (
                motorsCurrent, minimumElevatorAngle, maximumElevatorAngle,
                gravityPointAngle
        );
    }

    private static class Conventions {
        private final RotationalDirection mElevatorAngleMotorsPositiveDirection = RotationalDirection.Clockwise;

        final DeviceConventions elevatorAngleDeviceConventions = new DeviceConventions(
                mElevatorAngleMotorsPositiveDirection
        );

    }

    private static class Structure {
        private final GearRatio elevatorAngleMotorGR = new GearRatio(360, 1, 0);
        private final Angle encoderOffset = Degrees.of(0.0);
        final PhysicalDescription elevatorAnglePhysicalDescription = new PhysicalDescription(
                elevatorAngleMotorGR, encoderOffset
        );
    }

    private static class Control {
        private final PidfCoefficients elevatorAnglePidfCoefficients = new PidfCoefficients(0.0, 0.0, 0.0, 0.0);

        // Svag gains when we have no gravity Slot0
        private final SvagGains elevatorAngleNoGravitySvagGains = new SvagGains(0.0, 0.0, 0.01, 0.0);

        // Svag gains when we have gravity Slot1
        private final SvagGains elevatorAngleGravitySvagGains = new SvagGains(0.0, 0.0, 0.01, 0.0);

        private final MotionMagicSettings elevatorAngleMotionMagicCoefficients = new MotionMagicSettings(0.0, 0.0, 0.0);
        private final Time elevatorAngleRampRate = Seconds.of(0.1);

        final ControlConstants elevatorAngleControlConstants = new ControlConstants(
                elevatorAnglePidfCoefficients, elevatorAngleNoGravitySvagGains,
                elevatorAngleGravitySvagGains, elevatorAngleMotionMagicCoefficients,
                elevatorAngleRampRate
        );

    }

    public static final Config elevatorJointConfig = new Config(
            new DeviceIDS().elevatorAngleDevicesIDs, new Devices().elevatorAngleDeviceProperties,
            new Limits().elevatorAngleDeviceLimits, new Conventions().elevatorAngleDeviceConventions,
            new Structure().elevatorAnglePhysicalDescription, new Control().elevatorAngleControlConstants
    );
}