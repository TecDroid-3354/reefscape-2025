package net.tecdroid.subsystems.LimeLight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import kotlin.jvm.functions.Function0;
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver;

public class LimeLightsController {
    LimeLightModule leftLimeLight;
    LimeLightModule rightLimeLight;
    PIDController zAxisPIDController;
    PIDController yAxisPIDController;


    public LimeLightsController() {
        leftLimeLight = new LimeLightModule(LimeLightConfiguration.leftDeviceConfig);
        rightLimeLight = new LimeLightModule(LimeLightConfiguration.rightDeviceConfig);
        zAxisPIDController = new PIDController(0.01, 0.0, 0.0);
        yAxisPIDController = new PIDController(0.01, 0.0, 0.0);

    }

    /**
     * Gets the distance of the apriltag according to left camera
     * @param targetDistance distance from the target to the floor
     * @return the distance of the robot and the apriltag according to left camera
     */
    public Distance getLeftLimeLightDistance(Distance targetDistance) {
        return leftLimeLight.getDistance(targetDistance);
    }

    /**
     * Gets the distance of the apriltag according to right camera
     * @param targetDistance distance from the target to the floor
     * @return the distance of the robot and the apriltag according to right camera
     */
    public Distance getRightLimeLightDistance(Distance targetDistance) {
        return rightLimeLight.getDistance(targetDistance);
    }

    /**
     * Gets the distance average of the apriltag according to both cameras
     * @param targetDistance distance from the target to the floor
     * @return the distance of the robot and the apriltag according to both cameras
     */
    public Distance getLimeLightAverageDistance(Distance targetDistance) {
        if (leftLimeLight.hasTarget() && rightLimeLight.hasTarget()) {
            double distanceAverage = (getLeftLimeLightDistance(targetDistance).in(Units.Inches)
                    + getRightLimeLightDistance(targetDistance).in(Units.Inches)) / 2;
            return Distance.ofBaseUnits(distanceAverage, Units.Inches);
        } else if (leftLimeLight.hasTarget() && !rightLimeLight.hasTarget()) {
            return leftLimeLight.getDistance(targetDistance);
        } else if (!leftLimeLight.hasTarget() && rightLimeLight.hasTarget()) {
            return rightLimeLight.getDistance(targetDistance);
        } else {
            return Distance.ofBaseUnits(0, Units.Inches);
        }
    }

    public Angle getLeftLimeLightTx() {
        return leftLimeLight.getTx();
    }

    public Angle getRightLimeLightTx() {
        return rightLimeLight.getTx();
    }

    /**
     * Rotate the robot to the apriltag until align at the setpoint
     * @param swerveDriveDriver the driving subsystems
     * @param setPoint the angle setpoint
     * @param usingRight if true, you align the robot to the right camera, if false, to the left
     */

    public Command alignZAxisToAprilTagDetection(SwerveDriveDriver swerveDriveDriver, Angle setPoint,
                                                 Boolean usingRight) {
        return Commands.run(() -> {

            Angle tx = usingRight ? getRightLimeLightTx() : getLeftLimeLightTx();

            double targetingAngularVelocityFactor = zAxisPIDController.calculate(
                    tx.in(Units.Degrees), setPoint.in(Units.Degrees));

            swerveDriveDriver.setAngularVelocityFactorSource(() -> targetingAngularVelocityFactor);
        });
    }

    /**
     * Move the robot to the apriltag until be at the setpoint
     * @param swerveDriveDriver the driving subsystems
     * @param setpoint the distance setpoint
     * @param targetDistance distance from the target to the floor
     * @param usingRight if true, you align the robot to the right camera, if false, to the left
     */
    public Command alignYAxisToAprilTagDetection(SwerveDriveDriver swerveDriveDriver, Distance setpoint,
                                                 Distance targetDistance, Boolean usingRight) {
        return Commands.run(() -> {
            Distance aprilTagDistance = usingRight ? getRightLimeLightDistance(targetDistance) : getLeftLimeLightDistance(targetDistance);

            double targetingLinearVelocityFactor = yAxisPIDController.calculate(
                    getLimeLightAverageDistance(targetDistance).in(Units.Inches), setpoint.in(Units.Inches));

            swerveDriveDriver.setLongitudinalVelocityFactorSource(() -> targetingLinearVelocityFactor);
        });
    }

    /**
     * Move the robot to the apriltag until be at the setpoint
     * @param swerveDriveDriver the driving subsystems
     * @param setpoint the distance setpoint
     * @param targetDistance distance from the target to the floor
     */
    public Command alignYAxisToAprilTagDetectionUsingBothCameras(SwerveDriveDriver swerveDriveDriver, Distance setpoint,
                                                 Distance targetDistance) {
        return Commands.run(() -> {
            double targetingLinearVelocityFactor = yAxisPIDController.calculate(
                    getLimeLightAverageDistance(targetDistance).in(Units.Inches), setpoint.in(Units.Inches));

            swerveDriveDriver.setLongitudinalVelocityFactorSource(() -> targetingLinearVelocityFactor);
        });
    }

}