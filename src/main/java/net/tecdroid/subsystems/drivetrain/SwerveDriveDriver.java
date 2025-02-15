package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class SwerveDriveDriver {
    private static final double CONTROLLER_DEADBAND = 0.1;

    public enum DriveOrientation {
        FIELD_ORIENTED, ROBOT_ORIENTED
    }

    DriveOrientation orientation = DriveOrientation.ROBOT_ORIENTED;

    Supplier<LinearVelocity> forwardsVelocitySupplier;
    Supplier<LinearVelocity> sidewaysVelocitySupplier;
    Supplier<AngularVelocity> angularVelocitySupplier;


    public SwerveDriveDriver() {
        this.forwardsVelocitySupplier = MetersPerSecond::zero;
        this.sidewaysVelocitySupplier = MetersPerSecond::zero;
        this.angularVelocitySupplier  = RadiansPerSecond::zero;
    }

    public ChassisSpeeds obtainTargetSpeeds(Rotation2d currentAngle) {
        LinearVelocity vx = forwardsVelocitySupplier.get();
        LinearVelocity vy = sidewaysVelocitySupplier.get();
        AngularVelocity w = angularVelocitySupplier.get();

        return isFieldOriented() ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, w, currentAngle)
                                 : new ChassisSpeeds(vx, vy, w);
    }

    public void toggleOrientation() {
        orientation = switch (orientation) {
            case FIELD_ORIENTED -> DriveOrientation.ROBOT_ORIENTED;
            case ROBOT_ORIENTED -> DriveOrientation.FIELD_ORIENTED;
        };
    }

    private boolean isFieldOriented() {
        return orientation == DriveOrientation.FIELD_ORIENTED;
    }

    public void setForwardsVelocitySupplier(Supplier<LinearVelocity> forwardsVelocitySupplier) {
        this.forwardsVelocitySupplier = forwardsVelocitySupplier;
    }

    public void setSidewaysVelocitySupplier(Supplier<LinearVelocity> sidewaysVelocitySupplier) {
        this.sidewaysVelocitySupplier = sidewaysVelocitySupplier;
    }

    public void setAngularVelocitySupplier(Supplier<AngularVelocity> angularVelocitySupplier) {
        this.angularVelocitySupplier = angularVelocitySupplier;
    }
}