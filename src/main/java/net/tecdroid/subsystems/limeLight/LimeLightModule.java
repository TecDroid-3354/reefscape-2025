package net.tecdroid.subsystems.limeLight;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class LimeLightModule {
    private String limeLightName = "";
    private final DeviceConfig deviceConfig;


    public LimeLightModule(DeviceConfig deviceConfig) {
        this.deviceConfig = deviceConfig;
        this.limeLightName = deviceConfig.DeviceName.limeLightName;
    }

    /*public int getDetectionId() {
        return (int) LimelightHelpers.getFiducialID(limeLightName);
    }*/

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limeLightName);
    }

    public Angle getTx() {
        double tx = LimelightHelpers.getTX(limeLightName);  // Horizontal offset from crosshair to target in degrees
        return Angle.ofBaseUnits(tx, Units.Degrees);
    }

    public Angle getTy() {
        double ty = LimelightHelpers.getTY(limeLightName); // Vertical offset from crosshair to target in degrees
        return Angle.ofBaseUnits(ty, Units.Degrees);
    }

    public Distance getDistance() {
        if (hasTarget()) {
            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleRadians = deviceConfig.deviceStructure.limelightMountAngleDegrees
                    .in(Units.Radians);

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightInches = deviceConfig.deviceStructure.limelightLensHeightDistance
                    .in(Units.Inches);

            // distance from the target to the floor
            double goalHeightInches = deviceConfig.deviceStructure.goalHeightDistance.in(Units.Inches);

            double angleToGoalRadians = limelightMountAngleRadians + getTy().in(Units.Radians);

            //calculate distance
            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

            return Units.Inches.of(distanceFromLimelightToGoalInches);
        } else {
            return Units.Inches.of(0.0);
        }

    }

    public record DeviceName(String limeLightName) {}
    public record DeviceMeasures(Angle limelightMountAngleDegrees, Distance limelightLensHeightDistance,
                                 Distance goalHeightDistance) {}

    public record DeviceConfig(DeviceName DeviceName, DeviceMeasures deviceStructure) {}
}