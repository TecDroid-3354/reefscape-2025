package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class SwerveDriveDriver {

    Supplier<LinearVelocity>  forwardsVelocitySupplier;
    Supplier<LinearVelocity>  sidewaysVelocitySupplier;
    Supplier<AngularVelocity> angularVelocitySupplier;
    DriveOrientation orientation = DriveOrientation.FIELD_ORIENTED;

    public SwerveDriveDriver() {
        this.forwardsVelocitySupplier = MetersPerSecond::zero;
        this.sidewaysVelocitySupplier = MetersPerSecond::zero;
        this.angularVelocitySupplier = RadiansPerSecond::zero;
    }

    public ChassisSpeeds obtainTargetSpeeds(Rotation2d currentAngle) {
        LinearVelocity  vx = forwardsVelocitySupplier.get();
        LinearVelocity  vy = sidewaysVelocitySupplier.get();
        AngularVelocity w  = angularVelocitySupplier.get();

        return isFieldOriented() ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, w, currentAngle) :
            new ChassisSpeeds(vx, vy, w);
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

    public void setLongitudinalVelocitySupplier(Supplier<LinearVelocity> forwardsVelocitySupplier) {
        this.forwardsVelocitySupplier = forwardsVelocitySupplier;
    }

    public void setTransversalVelocitySupplier(Supplier<LinearVelocity> sidewaysVelocitySupplier) {
        this.sidewaysVelocitySupplier = sidewaysVelocitySupplier;
    }

    public void setAngularVelocitySupplier(Supplier<AngularVelocity> angularVelocitySupplier) {
        this.angularVelocitySupplier = angularVelocitySupplier;
    }

    public enum DriveOrientation {
        FIELD_ORIENTED,
        ROBOT_ORIENTED
    }
}