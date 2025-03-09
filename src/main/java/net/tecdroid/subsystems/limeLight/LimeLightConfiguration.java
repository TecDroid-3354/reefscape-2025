package net.tecdroid.subsystems.limeLight;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class LimeLightConfiguration {
    public static class LimeLightNames {
        private final String leftLimelightName = "leftLimeLight";
        private final String rightLimelightName = "rightLimeLight";

        LimeLightModule.DeviceName leftName = new LimeLightModule.DeviceName(leftLimelightName);
        LimeLightModule.DeviceName rightName = new LimeLightModule.DeviceName(rightLimelightName);


    }

    public static class LimeLightMeasures {
        // how many degrees back is your limelight rotated from perfectly vertical?
        private final Angle limelightMountAngleDegrees = Angle.ofBaseUnits(25.0, Units.Degrees);

        // distance from the center of the Limelight lens to the floor
        private final Distance limelightLensHeightDistance = Distance.ofBaseUnits(20.0, Units.Inches);

        // distance from the target to the floor
        private final Distance goalHeightDistance = Distance.ofBaseUnits(17.0, Units.Centimeters);

        LimeLightModule.DeviceMeasures deviceStructure = new LimeLightModule.DeviceMeasures(limelightMountAngleDegrees, limelightLensHeightDistance, goalHeightDistance);


    }

    static final LimeLightModule.DeviceConfig leftDeviceConfig = new LimeLightModule.DeviceConfig(
            new LimeLightNames().leftName,
            new LimeLightMeasures().deviceStructure);

    static final LimeLightModule.DeviceConfig rightDeviceConfig = new LimeLightModule.DeviceConfig(
            new LimeLightNames().rightName,
            new LimeLightMeasures().deviceStructure);

}