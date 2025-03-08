package frc.robot.subsystems.LimeLight;

import edu.wpi.first.units.Units;

public class LimeLightConfiguration {
    public static class LimeLightNames{
        private final String leftLimelightName = "leftLimeLight";
        private final String rightLimelightName = "rightLimeLight";

    LimeLightController.DeviceNames deviceNames = new LimeLightController.DeviceNames(leftLimelightName, rightLimelightName);

public class LimeLightStructure {
    public static class LimeLightMeasures {
        private final Double limelightMountAngleDegrees = 25.0;
        private final Double limelightLensHeightInches = 20.0;
        private final Double goalHeightInches = 60.0;

        LimeLightController.DeviceStructure deviceStructure = new LimeLightController.DeviceStructure(limelightMountAngleDegrees,limelightLensHeightInches, goalHeightInches);


    }

}

    }
}
