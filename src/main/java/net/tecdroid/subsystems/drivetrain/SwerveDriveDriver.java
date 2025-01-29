package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.joml.Vector2d;

import java.util.function.Supplier;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveUtil.denormalizeLinearVelocity;

public class SwerveDriveDriver {
    private static final double CONTROLLER_DEADBAND = 0.75;

    public enum DriveOrientation {
        FIELD_ORIENTED, ROBOT_ORIENTED
    }

    Supplier<Vector2d> leftJoystick;
    Supplier<Vector2d> rightJoystick;

    Rotation2d previousDirection = new Rotation2d();

    DriveOrientation orientation = DriveOrientation.FIELD_ORIENTED;

    public SwerveDriveDriver(Supplier<Vector2d> linearVelocitySource, Supplier<Vector2d> rightJoystick) {
        this.leftJoystick  = linearVelocitySource;
        this.rightJoystick = rightJoystick;
    }

    private ChassisSpeeds obtainTargetSpeeds(Rotation2d currentAngle) {
        Vector2d left = lstickAsLvec();

        double vx = denormalizeLinearVelocity(left.x());
        double vy = denormalizeLinearVelocity(left.y());

        return isFieldOriented() ?
               ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0.0, currentAngle)
                                 : new ChassisSpeeds(vx, vy, 0.0);
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
        ChassisSpeeds speeds    = obtainTargetSpeeds((Rotation2d) subsystem.getHeading());
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
        return new Vector2d(-left.y(), -left.x());
    }

    private Vector2d rstickAsRvec() {
        Vector2d right = rightJoystick.get();
        return new Vector2d(-right.y(), right.x());
    }

}