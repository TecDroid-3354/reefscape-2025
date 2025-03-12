package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import net.tecdroid.input.CompliantXboxController;
import net.tecdroid.subsystems.limeLight.LimeLightConfiguration;
import net.tecdroid.subsystems.limeLight.LimeLightModule;

public class LimeLightsController {
    LimeLightModule leftLimeLight;
    LimeLightModule rightLimeLight;
    PIDController zAxisPIDController;
    PIDController yAxisPIDController;
    PIDController xAxisPIDController;

    public LimeLightsController() {
        leftLimeLight = new LimeLightModule(LimeLightConfiguration.leftDeviceConfig);
        rightLimeLight = new LimeLightModule(LimeLightConfiguration.rightDeviceConfig);
        zAxisPIDController = new PIDController(0.0001, 0.0, 0.000001);
        yAxisPIDController = new PIDController(0.0005, 0.0, 0.000001);
        xAxisPIDController = new PIDController(0.0001, 0.0, 0.000001);

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

    public Angle getLeftLimeLightTx() {
        return leftLimeLight.getTx();
    }

    public Angle getRightLimeLightTx() {
        return rightLimeLight.getTx();
    }

    public Angle getLeftLimeLightTy() {
        return leftLimeLight.getTy();
    }

    public Angle getRightLimeLightTy() {
        return rightLimeLight.getTy();
    }

    public void publishShuffleBoard() {
        ShuffleboardTab tab = Shuffleboard.getTab("LimeLights");

        // X angle
        tab.addDouble("Right Tx", () -> getRightLimeLightTx().in(Units.Degrees));
        tab.addDouble("Left Tx", () -> getLeftLimeLightTx().in(Units.Degrees));

        // Distance
        tab.addDouble("Right Distance", () -> getRightLimeLightDistance(Units.Centimeters.of(29.0)).in(Units.Inches));
        tab.addDouble("Left Distance", () -> getLeftLimeLightDistance(Units.Centimeters.of(29.0)).in(Units.Inches));
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
                    tx.in(Units.Degrees), setPoint.in(Units.Degrees)) * 0.75;

            swerveDriveDriver.setAngularVelocityFactorSource(() -> targetingAngularVelocityFactor);
            swerveDriveDriver.setRobotOriented();
        });
    }

    /**
     * Rotate the robot to the apriltag until align at the setpoint
     * @param swerveDriveDriver the driving subsystems
     * @param setPoint the angle setpoint
     * @param usingRight if true, you align the robot to the right camera, if false, to the left
     */

    public Command alignXAxisToAprilTagDetection(SwerveDriveDriver swerveDriveDriver, Angle setPoint,
                                                 Boolean usingRight) {
        return Commands.run(() -> {
            Angle tx = usingRight ? getRightLimeLightTx() : getLeftLimeLightTx();

            double targetingTransversalVelocityFactor = -xAxisPIDController.calculate(
                    tx.in(Units.Degrees), setPoint.in(Units.Degrees)) * 0.75;

            swerveDriveDriver.setTransversalVelocityFactorSource(() -> targetingTransversalVelocityFactor);
            swerveDriveDriver.setRobotOriented();
        });
    }

    public Command alignYAxisToAprilTagDetectionWithTy(SwerveDriveDriver swerveDriveDriver, Angle setpoint, Boolean usingRight) {
        return Commands.run(() -> {
            Angle ty = usingRight ? getRightLimeLightTy() : getLeftLimeLightTy();

            double targetingLinearVelocityFactor = yAxisPIDController.calculate(
                    ty.in(Units.Degrees), setpoint.in(Units.Degrees));

            swerveDriveDriver.setLongitudinalVelocityFactorSource(() -> targetingLinearVelocityFactor * 0.75);
            swerveDriveDriver.setRobotOriented();
        });
    }

    public Command alignInAllAxis(SwerveDriveDriver swerveDriveDriver, Angle xSetPoint, Angle ySetPoint,
                                  Angle zSetPoint, Boolean usingRight, CompliantXboxController controller) {
        return Commands.run(() -> {
            Angle ty = usingRight ? getRightLimeLightTy() : getLeftLimeLightTy();
            Angle tx = usingRight ? getRightLimeLightTx() : getLeftLimeLightTx();

            double targetingLinearVelocityFactor = yAxisPIDController.calculate(
                    ty.in(Units.Degrees), ySetPoint.in(Units.Degrees));

            double targetingTransversalVelocityFactor = -xAxisPIDController.calculate(
                    tx.in(Units.Degrees), xSetPoint.in(Units.Degrees)) * 0.5;

            double targetingAngularVelocityFactor = zAxisPIDController.calculate(
                    tx.in(Units.Degrees), zSetPoint.in(Units.Degrees)) * 0.75;

            double joystickRxVelocity = controller.getRightX() * 0.25;

            swerveDriveDriver.setLongitudinalVelocityFactorSource(() -> targetingLinearVelocityFactor);
            swerveDriveDriver.setTransversalVelocityFactorSource(() -> targetingTransversalVelocityFactor);
            swerveDriveDriver.setAngularVelocityFactorSource(() -> targetingAngularVelocityFactor + joystickRxVelocity);
            swerveDriveDriver.setRobotOriented();
        });
    }

}