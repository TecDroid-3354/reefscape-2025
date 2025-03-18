package net.tecdroid.vision.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import net.tecdroid.constants.StringConstantsKt;
import net.tecdroid.util.ControlGains;
import net.tecdroid.util.LimeLightChoice;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class LimelightController {
    private final Limelight leftLimelight = new Limelight(new LimelightConfig(StringConstantsKt.leftLimelightName, new Translation3d()));
    private final Limelight rightLimelight = new Limelight(new LimelightConfig(StringConstantsKt.rightLimelightName, new Translation3d()));

    private final ControlGains xGains = new ControlGains(0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    private final ControlGains yGains = new ControlGains(0.45, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    private final ControlGains thetaGains = new ControlGains(0.0075, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    private final PIDController xPIDController = new PIDController(xGains.getP(), xGains.getI(), xGains.getD());
    private final PIDController yPIDController = new PIDController(yGains.getP(), yGains.getI(), yGains.getD());
    private final PIDController thetaPIDController = new PIDController(thetaGains.getP(), thetaGains.getI(), thetaGains.getD());

    Map<Integer, Double> alignmentAngles = new HashMap<>();
    private final Consumer<double[]> drive;

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
    public LimelightController(Consumer<double[]> drive) {
        this.drive = drive;
        thetaPIDController.enableContinuousInput(0.0, 360.0);
        angleDictionaryValues();

    }

    public Pose3d getRobotPositionInTargetSpace(LimeLightChoice choice) {
        Limelight limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getRobotPositionInTargetSpace();
    }

    public int getTargetId(LimeLightChoice choice) {
        Limelight limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getTargetId();
    }

    public boolean hasTarget(LimeLightChoice choice) {
        Limelight limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getHasTarget();
    }

    public Command alignRobotXAxis(LimeLightChoice choice, double setPoint) {
        return Commands.run(() -> {
            Pose3d robotPose = getRobotPositionInTargetSpace(choice);
            double driveXVelocity = xPIDController.calculate(robotPose.getTranslation().getX(), setPoint);
            double[] velocities = {driveXVelocity, 0.0, 0.0};
            drive.accept(velocities);

        }).onlyIf(() -> hasTarget(choice));
    }

    public Command alignRobotYAxis(LimeLightChoice choice, double setPoint) {
        return Commands.run(() -> {
            Pose3d robotPose = getRobotPositionInTargetSpace(choice);

            double driveYVelocity = yPIDController.calculate(robotPose.getTranslation().getY(), setPoint);
            double[] velocities = {0.0, driveYVelocity, 0.0};
            drive.accept(velocities);

        }).onlyIf(() -> hasTarget(choice));
    }

    public Command alignRobotThetaAxis(LimeLightChoice choice, Angle yaw) {
        return Commands.run(() -> {
            double yawSetPoint = alignmentAngles.get(getTargetId(choice));

            double driveThetaVelocity = thetaPIDController.calculate(yaw.in(Units.Degrees), yawSetPoint);
            double[] velocities = {0.0, 0.0, driveThetaVelocity};
            drive.accept(velocities);

        }).onlyIf(() -> hasTarget(choice));
    }

    public Command alignRobotAllAxis(LimeLightChoice choice, Angle yaw, double xSetPoint, double ySetPoint) {
        return Commands.run(() -> {
            Pose3d robotPose = getRobotPositionInTargetSpace(choice);
            double yawSetPoint = alignmentAngles.get(getTargetId(choice));

            double driveXVelocity = xPIDController.calculate(robotPose.getTranslation().getX(), xSetPoint);
            double driveYVelocity = yPIDController.calculate(robotPose.getTranslation().getY(), ySetPoint);
            double driveThetaVelocity = thetaPIDController.calculate(yaw.in(Units.Degrees), yawSetPoint);

            double[] velocities = {driveXVelocity, driveYVelocity, driveThetaVelocity};
            drive.accept(velocities);
        }).onlyIf(() -> hasTarget(choice));
    }

    public void shuffleboardData() {
        ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

        tab.add("lPosition", getRobotPositionInTargetSpace(LimeLightChoice.Left));
        tab.add("rPosition", getRobotPositionInTargetSpace(LimeLightChoice.Right));
    }


}
