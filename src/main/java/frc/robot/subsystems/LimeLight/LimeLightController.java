package frc.robot.subsystems.LimeLight;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimeLightController extends SubsystemBase {
    private final String limeLightName = "";

    public LimeLightController() {

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
        double limelightMountAngleDegrees = 25.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0;

        // distance from the target to the floor
        double goalHeightInches = 60.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + getTy().in(Units.Degrees);
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        return Distance.ofBaseUnits(distanceFromLimelightToGoalInches, Units.Inches);
    }

    public record DeviceNames(String leftLimeLightName, String  rightLimeLightName) {}
    public record DeviceStructure(Double leftLimeLightHeightInches, Double rightLimeLightHeightInches,
                                  Double goalHeightInches) {}
}
