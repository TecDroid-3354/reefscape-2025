package net.tecdroid.subsystems.LimeLight;

public class LimeLightConfiguration {
    public static class LimeLightNames {
        private final String leftLimelightName = "leftLimeLight";
        private final String rightLimelightName = "rightLimeLight";

        LimeLightModule.DeviceName leftName = new LimeLightModule.DeviceName(leftLimelightName);
        LimeLightModule.DeviceName rightName = new LimeLightModule.DeviceName(rightLimelightName);


    }

    public static class LimeLightMeasures {
        // how many degrees back is your limelight rotated from perfectly vertical?
        private final Double limelightMountAngleDegrees = 25.0;

        // distance from the center of the Limelight lens to the floor
        private final Double limelightLensHeightInches = 20.0;

        // distance from the target to the floor
        private final Double goalHeightInches = 60.0;

        LimeLightModule.DeviceStructure deviceStructure = new LimeLightModule.DeviceStructure(limelightMountAngleDegrees,limelightLensHeightInches, goalHeightInches);


    }

    static final LimeLightModule.DeviceConfig leftDeviceConfig = new LimeLightModule.DeviceConfig(
            new LimeLightNames().leftName,
            new LimeLightMeasures().deviceStructure);

    static final LimeLightModule.DeviceConfig rightDeviceConfig = new LimeLightModule.DeviceConfig(
            new LimeLightNames().rightName,
            new LimeLightMeasures().deviceStructure);

}