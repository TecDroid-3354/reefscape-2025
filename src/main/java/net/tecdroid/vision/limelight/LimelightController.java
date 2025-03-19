package net.tecdroid.vision.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.tecdroid.constants.StringConstantsKt;
import net.tecdroid.util.ControlGains;
import net.tecdroid.util.LimeLightChoice;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class LimelightController {
    private final LimelightAprilTagDetector leftLimelight = new LimelightAprilTagDetector(new LimelightConfig(StringConstantsKt.leftLimelightName, new Pose3d()));
    private final LimelightAprilTagDetector rightLimelight = new LimelightAprilTagDetector(new LimelightConfig(StringConstantsKt.rightLimelightName, new Pose3d()));

    private final Subsystem requiredSubsystem;

    private final ControlGains xGains = new ControlGains(0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    private final ControlGains yGains = new ControlGains(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    private final ControlGains thetaGains = new ControlGains(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    private final PIDController xPIDController = new PIDController(xGains.getP(), xGains.getI(), xGains.getD());
    private final PIDController yPIDController = new PIDController(yGains.getP(), yGains.getI(), yGains.getD());
    private final PIDController thetaPIDController = new PIDController(thetaGains.getP(), thetaGains.getI(), thetaGains.getD());

    private Map<Integer, Double> alignmentAngles = new HashMap<>();
    private final Consumer<ChassisSpeeds> drive;
    private final DoubleSupplier yaw;

    public void angleDictionaryValues() {
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

    public LimelightController(Subsystem requiredSubsystem, Consumer<ChassisSpeeds> drive, DoubleSupplier yaw) {
        this.requiredSubsystem = requiredSubsystem;
        this.drive = drive;
        this.yaw = yaw;

        thetaPIDController.enableContinuousInput(0.0, 360.0);
        angleDictionaryValues();

    }

    public Pose3d getRobotPositionInTargetSpace(LimeLightChoice choice) {
        LimelightAprilTagDetector limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getRobotPositionInTargetSpace().getPose3d();
    }


    public int getTargetId(LimeLightChoice choice) {
        LimelightAprilTagDetector limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getTargetId();
    }

    public boolean hasTarget(LimeLightChoice choice) {
        LimelightAprilTagDetector limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getHasTarget();
    }

    // Obtain a yaw between the range [0, 360]
    public double getLimitedYaw() {
        double limitedYaw = yaw.getAsDouble() % 360;
        if (limitedYaw < 0) {
            limitedYaw += 360;
        }
        return limitedYaw;
    }

    public Command alignRobotXAxis(LimeLightChoice choice, double setPoint) {
        return Commands.run(() -> {
            Pose3d robotPose = getRobotPositionInTargetSpace(choice);
            double driveXVelocity = xPIDController.calculate(robotPose.getTranslation().getX(), setPoint);
            drive.accept(new ChassisSpeeds(driveXVelocity, 0.0, 0.0));

        }, requiredSubsystem).onlyIf(() -> hasTarget(choice));
    }

    public Command alignRobotYAxis(LimeLightChoice choice, double setPoint) {
        return Commands.run(() -> {
            Pose3d robotPose = getRobotPositionInTargetSpace(choice);
            double driveYVelocity = -yPIDController.calculate(robotPose.getTranslation().getZ(), setPoint);
            drive.accept(new ChassisSpeeds(driveYVelocity, 0.0, 0.0));

        }, requiredSubsystem).onlyIf(() -> hasTarget(choice));
    }

    public Command alignRobotThetaAxis(LimeLightChoice choice) {
        return Commands.run(() -> {
            if (alignmentAngles.containsKey(getTargetId(choice))) {
                double yawSetPoint = alignmentAngles.get(getTargetId(choice));
                double driveThetaVelocity = thetaPIDController.calculate(getLimitedYaw(), yawSetPoint);
                drive.accept(new ChassisSpeeds(0.0, 0.0, driveThetaVelocity));
            }
        }, requiredSubsystem).onlyIf(() -> hasTarget(choice));
    }

    public Command alignRobotAllAxis(LimeLightChoice choice, double xSetPoint, double ySetPoint) {
        return Commands.run(() -> {
            Pose3d robotPose = getRobotPositionInTargetSpace(choice);

            double driveXVelocity = xPIDController.calculate(robotPose.getTranslation().getX(), xSetPoint);
            double driveYVelocity = yPIDController.calculate(robotPose.getTranslation().getY(), ySetPoint);

            if (alignmentAngles.containsKey(getTargetId(choice))) {
                double yawSetPoint = alignmentAngles.get(getTargetId(choice));
                double driveThetaVelocity = thetaPIDController.calculate(getLimitedYaw(), yawSetPoint);
                drive.accept(new ChassisSpeeds(driveYVelocity, driveXVelocity, driveThetaVelocity));
            } else {
                drive.accept(new ChassisSpeeds(driveYVelocity, driveXVelocity, 0.0));
            }
        }, requiredSubsystem).onlyIf(() -> hasTarget(choice));
    }

    public void shuffleboardData() {
        ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

        tab.addDoubleArray("rPosition", () -> new double[]{getRobotPositionInTargetSpace(LimeLightChoice.Right).getX(), getRobotPositionInTargetSpace(LimeLightChoice.Right).getY(), getRobotPositionInTargetSpace(LimeLightChoice.Right).getZ()});
        tab.addDoubleArray("lPosition", () -> new double[]{getRobotPositionInTargetSpace(LimeLightChoice.Left).getX(), getRobotPositionInTargetSpace(LimeLightChoice.Left).getY(), getRobotPositionInTargetSpace(LimeLightChoice.Left).getZ()});
        tab.addDouble("RobotYaw", this::getLimitedYaw);
        tab.addInteger("Yaw Objective", rightLimelight::getTargetId);
    }


}