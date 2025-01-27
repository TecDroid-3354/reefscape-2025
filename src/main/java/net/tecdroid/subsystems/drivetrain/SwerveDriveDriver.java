package net.tecdroid.subsystems.drivetrain;

import java.util.function.Supplier;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static net.tecdroid.subsystems.drivetrain.SwerveDriveUtil.denormalizeLinearVelocity;

public class SwerveDriveDriver {
    private static final double CONTROLLER_DEADBAND = 0.75;

    public enum DriveOrientation {
        FIELD_ORIENTED, ROBOT_ORIENTED
    }

    Supplier<Pair<Double, Double>> leftJoystick;
    Supplier<Pair<Double, Double>> rightJoystick;

    Rotation2d previousDirection = new Rotation2d();

    DriveOrientation orientation = DriveOrientation.FIELD_ORIENTED;

    public SwerveDriveDriver(Supplier<Pair<Double, Double>> leftJoystick, Supplier<Pair<Double, Double>> rightJoystick) {
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;

        ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
        tab.addDouble("Driver Target Angle", () -> previousDirection.getDegrees());

    }

    private ChassisSpeeds obtainTargetSpeeds(Rotation2d currentAngle) {
        Pair<Double, Double> left = leftJoystick.get();

        // The x for the robot is the y for the controller
        double vx = denormalizeLinearVelocity(left.getSecond());
        // The y for the robot is the x for the controller
        double vy = denormalizeLinearVelocity(left.getFirst());

        if (orientation == DriveOrientation.FIELD_ORIENTED) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0.0, currentAngle);
        }

        return new ChassisSpeeds(vx, vy, 0.0);
    }

    private Rotation2d obtainTargetDirection() {
        Pair<Double, Double> right = rightJoystick.get();

        if (!isRotationTargetWithinDeadband(right.getFirst(), -right.getSecond())) {
            return previousDirection;
        }

        Rotation2d target = Rotation2d.fromRadians(atan2(right.getSecond(), right.getFirst()))
                                      .rotateBy(Rotation2d.fromDegrees(-90))
                                      .unaryMinus(); //Didn't have right the conventions, should delete it now
        previousDirection = target;

        return target;

    }


    public void apply(SwerveDrive subsystem) {
        ChassisSpeeds speeds = obtainTargetSpeeds(subsystem.getHeading());
        Rotation2d direction = obtainTargetDirection();

        SmartDashboard.putNumber("bbbbbbbb", previousDirection.getDegrees());


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



}