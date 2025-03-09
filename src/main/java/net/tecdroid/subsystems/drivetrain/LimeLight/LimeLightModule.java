package net.tecdroid.subsystems.drivetrain.LimeLight;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class LimeLightModule {
    private String limeLightName = "";
    private DeviceConfig deviceConfig;

    public LimeLightModule(DeviceConfig deviceConfig) {
        this.deviceConfig = deviceConfig;
        this.limeLightName = deviceConfig.DeviceName.limeLightName;
    }

    public int getDetectionId() {
        return (int)LimelightHelpers.getFiducialID(limeLightName);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limeLightName);
    }

    public Angle getTx() {
        double tx = LimelightHelpers.getTX(limeLightName);  // Horizontal offset from crosshair to target in degrees
        return Angle.ofBaseUnits(tx, Units.Degrees);
    }

    public Angle getTy() {
        double ty = LimelightHelpers.getTY(limeLightName);  // Vertical offset from crosshair to target in degrees
        return Angle.ofBaseUnits(ty, Units.Degrees);
    }

    public Angle getTa() {
        double ta = LimelightHelpers.getTA(limeLightName);  // Target area (0% to 100% of image)
        return Angle.ofBaseUnits(ta, Units.Degrees);
    }

    public Angle getTxnc() {
        double txnc = LimelightHelpers.getTXNC(limeLightName);  // Horizontal offset from principal pixel/point to target in degrees
        return Angle.ofBaseUnits(txnc, Units.Degrees);
    }

    public Angle getTync() {
        double tync = LimelightHelpers.getTYNC(limeLightName);  // Vertical  offset from principal pixel/point to target in degrees
        return Angle.ofBaseUnits(tync, Units.Degrees);
    }

    public Distance getDistance() {
        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = deviceConfig.deviceStructure.limelightMountAngleDegrees;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = deviceConfig.deviceStructure.limelightLensHeightInches;

        // distance from the target to the floor
        double goalHeightInches = deviceConfig.deviceStructure.goalHeightInches;

        double angleToGoalDegrees = limelightMountAngleDegrees + getTy().in(Units.Degrees);
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        return Distance.ofBaseUnits(distanceFromLimelightToGoalInches, Units.Inches);
    }

    public record DeviceName(String limeLightName) {}
    public record DeviceStructure(Double limelightMountAngleDegrees, Double limelightLensHeightInches,
                                  Double goalHeightInches) {}

    public record DeviceConfig(DeviceName DeviceName, DeviceStructure deviceStructure) {}
}