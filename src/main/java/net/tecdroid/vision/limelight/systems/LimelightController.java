package net.tecdroid.vision.limelight.systems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.tecdroid.constants.StringConstantsKt;
import net.tecdroid.util.ControlGains;
import net.tecdroid.util.LimeLightChoice;
import net.tecdroid.vision.limelight.Limelight;
import net.tecdroid.vision.limelight.LimelightAprilTagDetector;
import net.tecdroid.vision.limelight.LimelightConfig;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class LimelightController {
    private final LimelightAprilTagDetector leftLimelight  = new LimelightAprilTagDetector(new LimelightConfig(StringConstantsKt.leftLimelightName, new Pose3d()));
    private final LimelightAprilTagDetector rightLimelight = new LimelightAprilTagDetector(new LimelightConfig(StringConstantsKt.rightLimelightName, new Pose3d()));

    private final Subsystem requiredSubsystem;

    private final ControlGains xGains = new ControlGains(1.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    private final ControlGains yGains = new ControlGains(1.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    private final ControlGains thetaGains = new ControlGains(0.08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    private final PIDController xPIDController = new PIDController(xGains.getP(), xGains.getI(), xGains.getD());
    private final PIDController yPIDController = new PIDController(yGains.getP(), yGains.getI(), yGains.getD());
    private final PIDController thetaPIDController = new PIDController(thetaGains.getP(), thetaGains.getI(), thetaGains.getD());

    private final Map<Integer, Double> alignmentAngles = new HashMap<>();
    private final Consumer<ChassisSpeeds> drive;
    private final DoubleSupplier yaw;

    private final double positionTolerance = 0.2;

    private void angleDictionaryValues() {
        // Blue
        alignmentAngles.put(21, 0.0);
        alignmentAngles.put(20, 60.0);
        alignmentAngles.put(19, 120.0);
        alignmentAngles.put(18, 180.0);
        alignmentAngles.put(17, 240.0);
        alignmentAngles.put(22, 300.0);

        // Red
        alignmentAngles.put(10, 0.0);
        alignmentAngles.put(11, 60.0);
        alignmentAngles.put(6, 120.0);
        alignmentAngles.put(7, 180.0);
        alignmentAngles.put(8, 240.0);
        alignmentAngles.put(9, 300.0);
    }

    private void limelightConfiguration() {
        thetaPIDController.enableContinuousInput(0.0, 360.0);

        Integer[] validIDs = { 21, 20, 19, 18, 17, 22, 10, 11, 6, 7, 8, 9 };
        rightLimelight.setIdFilter(validIDs);
        leftLimelight.setIdFilter(validIDs);

    }

    private static double clamp(double max, double min, double v)
    {
        return Math.max(min, Math.min(max, v));
    }

    public LimelightController(Subsystem requiredSubsystem, Consumer<ChassisSpeeds> drive, DoubleSupplier yaw) {
        this.requiredSubsystem = requiredSubsystem;
        this.drive = drive;
        this.yaw = yaw;

        angleDictionaryValues();
        limelightConfiguration();
    }

    private boolean isAtSetPoint(LimeLightChoice choice, double xSetPoint, double ySetPoint) {
        Pose3d robotPose = getTargetPositionInCameraSpace(choice);
        double xDisplacement = Math.abs(xSetPoint - robotPose.getTranslation().getZ());
        double yDisplacement = Math.abs(ySetPoint - robotPose.getTranslation().getX());

        return xDisplacement <= positionTolerance && yDisplacement <= positionTolerance;
    }

    private Pose3d getTargetPositionInCameraSpace(LimeLightChoice choice) {
        LimelightAprilTagDetector limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getTargetPositionInCameraSpace();
    }


    private int getTargetId(LimeLightChoice choice) {
        LimelightAprilTagDetector limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getTargetId();
    }

    private boolean hasTarget(LimeLightChoice choice) {
        Limelight limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getHasTarget();
    }

    // Obtain a yaw between the range [0, 360]
    private double getLimitedYaw() {
        double limitedYaw = yaw.getAsDouble() % 360;
        if (limitedYaw < 0) {
            limitedYaw += 360;
        }
        return limitedYaw;
    }

    public Command alignRobotXAxis(LimeLightChoice choice, double setPoint) {
        return Commands.run(() -> {
            Pose3d robotPose = getTargetPositionInCameraSpace(choice);
            double driveXVelocity = xPIDController.calculate(robotPose.getTranslation().getZ(), setPoint);
            drive.accept(new ChassisSpeeds(driveXVelocity, 0.0, 0.0));

        }, requiredSubsystem).onlyIf(() -> hasTarget(choice));
    }

    public Command alignRobotYAxis(LimeLightChoice choice, double setPoint) {
        return Commands.run(() -> {
            Pose3d robotPose = getTargetPositionInCameraSpace(choice);
            double driveYVelocity = -yPIDController.calculate(robotPose.getTranslation().getX(), setPoint);
            drive.accept(new ChassisSpeeds(0.0, driveYVelocity, 0.0));

        }, requiredSubsystem).onlyIf(() -> hasTarget(choice));
    }

    public Command alignRobotThetaAxis(LimeLightChoice choice) {
        return Commands.run(() -> {
            Double yawSetPoint = alignmentAngles.get(getTargetId(choice));

            if (yawSetPoint != null) {
                double driveThetaVelocity = thetaPIDController.calculate(getLimitedYaw(), yawSetPoint);
                drive.accept(new ChassisSpeeds(0.0, 0.0, driveThetaVelocity));
            } else {
                drive.accept(new ChassisSpeeds(0.0, 0.0, 0.0));
            }
        }, requiredSubsystem).onlyIf(() -> hasTarget(choice));
    }

    public Command alignRobotAllAxis(LimeLightChoice choice, double xSetPoint, double ySetPoint) {
        return Commands.run(() -> {
            Pose3d robotPose = getTargetPositionInCameraSpace(choice);
            Double yawSetPoint = alignmentAngles.get(getTargetId(choice));

            double driveXVelocity = clamp(1.0, -1.0, xPIDController.calculate(robotPose.getTranslation().getZ(), xSetPoint));
            double driveYVelocity = -clamp(1.0, -1.0, yPIDController.calculate(robotPose.getTranslation().getX(), ySetPoint));

            if (!hasTarget(choice)) {
                drive.accept(new ChassisSpeeds(0.0, 0.0, 0.0));
            } else if (yawSetPoint != null) {
                double driveThetaVelocity = clamp(1.0, -1.0, thetaPIDController.calculate(getLimitedYaw(), yawSetPoint));
                drive.accept(new ChassisSpeeds(driveXVelocity, driveYVelocity, driveThetaVelocity));
            } else {
                drive.accept(new ChassisSpeeds(driveXVelocity, driveYVelocity, 0.0));
            }


        }, requiredSubsystem);
    }

    public Command alignRobotWithAprilTagChoice(LimeLightChoice choice, double xSetPoint, double ySetPoint, int aprilTagId) {
        return alignRobotAllAxis(choice, xSetPoint, ySetPoint).onlyIf(() -> getTargetId(choice) == aprilTagId);
    }

    public Command alignRobotAuto(LimeLightChoice choice, double xSetPoint, double ySetPoint, int aprilTagId, long waitMillis) throws InterruptedException {
        Command alignRobotWithAprilTagChoice = alignRobotWithAprilTagChoice(choice, xSetPoint, ySetPoint, aprilTagId);
        alignRobotWithAprilTagChoice.until(() -> isAtSetPoint(choice, xSetPoint, ySetPoint)).wait(waitMillis);
        return alignRobotWithAprilTagChoice;
    }

    public void shuffleboardData() {
        ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

        tab.addDoubleArray("rPosition", () -> new double[]{ getTargetPositionInCameraSpace(LimeLightChoice.Right).getX(), getTargetPositionInCameraSpace(LimeLightChoice.Right).getY(), getTargetPositionInCameraSpace(LimeLightChoice.Right).getZ()});
        tab.addDoubleArray("lPosition", () -> new double[]{ getTargetPositionInCameraSpace(LimeLightChoice.Left).getX(), getTargetPositionInCameraSpace(LimeLightChoice.Left).getY(), getTargetPositionInCameraSpace(LimeLightChoice.Left).getZ()});
        tab.addDouble("RobotYaw", this::getLimitedYaw);
        tab.addInteger("Yaw Objective", rightLimelight::getTargetId);
    }


}