package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.joml.Vector2d;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveUtil.denormalizeLinearVelocity;

public class SwerveDriveDriver {
    private static final double CONTROLLER_DEADBAND = 0.1;

    public enum DriveOrientation {
        FIELD_ORIENTED, ROBOT_ORIENTED
    }

    Supplier<Vector2d> leftJoystick;
    Supplier<Vector2d> rightJoystick;

    Rotation2d previousDirection = new Rotation2d();

    DriveOrientation orientation = DriveOrientation.ROBOT_ORIENTED;

    public SwerveDriveDriver(Supplier<Vector2d> linearVelocitySource, Supplier<Vector2d> rightJoystick) {
        this.leftJoystick  = linearVelocitySource;
        this.rightJoystick = rightJoystick;
    }

    private ChassisSpeeds obtainTargetSpeeds(Rotation2d currentAngle) {
        Vector2d left = lstickAsLvec();

        LinearVelocity vx = denormalizeLinearVelocity(left.x());
        LinearVelocity vy = denormalizeLinearVelocity(left.y());

        SmartDashboard.putNumber("Target vx", vx.in(MetersPerSecond));
        SmartDashboard.putNumber("Target vy", vy.in(MetersPerSecond));

        return isFieldOriented() ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, RadiansPerSecond.of(0.0), currentAngle)
                                 : new ChassisSpeeds(vx, vy, RadiansPerSecond.of(0.0));
    }

    private Rotation2d obtainTargetDirection(Vector2d direction) {
        if (!isRotationTargetWithinDeadband(direction.x(), direction.y())) {
            return previousDirection;
        }

        Rotation2d target = Rotation2d.fromRadians(atan2(direction.y(), direction.x()))
                                      .rotateBy(Rotation2d.fromDegrees(-90))
                                      .unaryMinus();
        previousDirection = target;

        return target;
    }

    public void apply(SwerveDrive subsystem) {
        ChassisSpeeds speeds    = obtainTargetSpeeds(new Rotation2d(subsystem.getHeading()));
        Rotation2d    direction = obtainTargetDirection(rstickAsRvec());

        subsystem.drive(speeds, direction);
    }

    public void toggleOrientation() {
        orientation = switch (orientation) {
            case FIELD_ORIENTED -> DriveOrientation.ROBOT_ORIENTED;
            case ROBOT_ORIENTED -> DriveOrientation.FIELD_ORIENTED;
        };
    }

    private boolean isRotationTargetWithinDeadband(double x, double y) {
        return abs(x) > CONTROLLER_DEADBAND || abs(y) > CONTROLLER_DEADBAND;
    }

    private boolean isFieldOriented() {
        return orientation == DriveOrientation.FIELD_ORIENTED;
    }

    private Vector2d lstickAsLvec() {
        Vector2d left = leftJoystick.get();
        double x = -left.y();
        double y = -left.x();
        return new Vector2d(abs(x) > CONTROLLER_DEADBAND ? x : 0, abs(y) > CONTROLLER_DEADBAND ? y : 0);
    }

    private Vector2d rstickAsRvec() {
        Vector2d right = rightJoystick.get();
        double x = -right.y();
        double y = -right.x();
        return new Vector2d(abs(x) > CONTROLLER_DEADBAND ? x : 0, abs(y) > CONTROLLER_DEADBAND ? y : 0);
    }

}