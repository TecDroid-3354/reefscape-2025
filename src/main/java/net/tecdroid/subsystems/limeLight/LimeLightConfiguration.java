package net.tecdroid.subsystems.limeLight;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class LimeLightConfiguration {
    public static class LimeLightNames {
        private final String leftLimelightName = "limelight-left";
        private final String rightLimelightName = "limelight-right";

        LimeLightModule.DeviceName leftName = new LimeLightModule.DeviceName(leftLimelightName);
        LimeLightModule.DeviceName rightName = new LimeLightModule.DeviceName(rightLimelightName);


    }

    public static class LimeLightMeasures {
        // how many degrees back is your limelight rotated from perfectly vertical?
        private final Angle limelightMountAngleDegrees = Units.Degrees.of(-15);

        // distance from the center of the Limelight lens to the floor
        private final Distance limelightLensHeightDistance = Units.Inches.of(15.1819);

        // distance from the target to the floor
        private final Distance goalHeightDistance = Distance.ofBaseUnits(30.5, Units.Centimeters);

        LimeLightModule.DeviceMeasures deviceStructure = new LimeLightModule.DeviceMeasures(limelightMountAngleDegrees, limelightLensHeightDistance, goalHeightDistance);


    }

    public static final LimeLightModule.DeviceConfig leftDeviceConfig = new LimeLightModule.DeviceConfig(
            new LimeLightNames().leftName,
            new LimeLightMeasures().deviceStructure);

    public static final LimeLightModule.DeviceConfig rightDeviceConfig = new LimeLightModule.DeviceConfig(
            new LimeLightNames().rightName,
            new LimeLightMeasures().deviceStructure);

}