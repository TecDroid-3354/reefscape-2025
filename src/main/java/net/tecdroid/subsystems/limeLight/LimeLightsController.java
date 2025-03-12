package net.tecdroid.subsystems.limeLight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver;

public class LimeLightsController {
    LimeLightModule leftLimeLight;
    LimeLightModule rightLimeLight;
    PIDController zAxisPIDController;
    PIDController yAxisPIDController;
    PIDController xAxisPIDController;


    public LimeLightsController() {
        leftLimeLight = new LimeLightModule(LimeLightConfiguration.leftDeviceConfig);
        rightLimeLight = new LimeLightModule(LimeLightConfiguration.rightDeviceConfig);
        zAxisPIDController = new PIDController(0.01, 0.0, 0.0);
        yAxisPIDController = new PIDController(0.01, 0.0, 0.0);
        xAxisPIDController = new PIDController(0.01, 0.0, 0.0);

    }

    public Distance getLeftLimeLightDistance() {
        return leftLimeLight.getDistance();
    }
    public Distance getRightLimeLightDistance() {
        return rightLimeLight.getDistance();
    }

    // Tx = limelight's horizontal offset
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

    public boolean isAlignedAtReef(boolean usingRight) {
        Angle tx = usingRight ? getRightLimeLightTx() : getLeftLimeLightTx();
        Distance aprilTagDistance = usingRight ? getRightLimeLightDistance() : getLeftLimeLightDistance();

        double xTolerance = Units.Degrees.of(5.0).in(Units.Degrees);
        double xSetPoint = Units.Degrees.of(0.0).in(Units.Degrees);

        double yTolerance = Units.Centimeters.of(5.0).in(Units.Centimeters);
        double ySetPoint = Units.Centimeters.of(0.0).in(Units.Centimeters);

        boolean alignedXAxis = xSetPoint + xTolerance > tx.in(Units.Degrees) && tx.in(Units.Degrees) > xSetPoint - xTolerance;
        boolean alignedYAxis = ySetPoint + yTolerance > aprilTagDistance.in(Units.Centimeters)
                && aprilTagDistance.in(Units.Centimeters) > ySetPoint - yTolerance;

        return alignedXAxis && alignedYAxis;
    }

    public Command alignZAxisToAprilTagDetection(SwerveDriveDriver swerveDriveDriver, Angle setPoint,
                                                 Boolean usingRight) {
        return Commands.run(() -> {

            Angle tx = usingRight ? getRightLimeLightTx() : getLeftLimeLightTx();

            double targetingAngularVelocityFactor = zAxisPIDController.calculate(
                    tx.in(Units.Degrees), setPoint.in(Units.Degrees));

            swerveDriveDriver.setAngularVelocityFactorSource(() -> targetingAngularVelocityFactor);
        });
    }

    public Command alignXAxisToAprilTagDetection(SwerveDriveDriver swerveDriveDriver, Angle setPoint,
                                                 Boolean usingRight) {
        return Commands.run(() -> {
            if (leftLimeLight.hasTarget() || rightLimeLight.hasTarget()) {
                Angle tx = usingRight ? getRightLimeLightTx() : getLeftLimeLightTx();

                double targetingTransversalVelocityFactor = -xAxisPIDController.calculate(
                        tx.in(Units.Degrees), setPoint.in(Units.Degrees)) * 0.5;

                swerveDriveDriver.setTransversalVelocityFactorSource(() -> targetingTransversalVelocityFactor);
            } else {
                swerveDriveDriver.setTransversalVelocityFactorSource(() -> 0.0);

            }
        });
    }

    public Command alignYAxisToAprilTagDetection(SwerveDriveDriver swerveDriveDriver, Distance setpoint, Boolean usingRight) {
        return Commands.run(() -> {
            Distance aprilTagDistance = usingRight ? getRightLimeLightDistance() : getLeftLimeLightDistance();

            double targetingLinearVelocityFactor = yAxisPIDController.calculate(
                    aprilTagDistance.in(Units.Inches), setpoint.in(Units.Inches));

            swerveDriveDriver.setLongitudinalVelocityFactorSource(() -> targetingLinearVelocityFactor);
        });
    }

    public Command alignYAxisToAprilTagDetectionWithTy(SwerveDriveDriver swerveDriveDriver, Angle setpoint, Boolean usingRight) {
        return Commands.run(() -> {
            Angle aprilTagDistance = usingRight ? getRightLimeLightTy() : getLeftLimeLightTy();

            double targetingLinearVelocityFactor = yAxisPIDController.calculate(
                    aprilTagDistance.in(Units.Degrees), setpoint.in(Units.Degrees));

            swerveDriveDriver.setLongitudinalVelocityFactorSource(() -> targetingLinearVelocityFactor * 0.5);
        });
    }

    public Command alignInAllAxis(SwerveDriveDriver swerveDriveDriver, Angle xSetPoint, Distance ySetPoint, Boolean usingRight) {
        return Commands.run(() -> {
            Distance aprilTagDistance = usingRight ? getRightLimeLightDistance() : getLeftLimeLightDistance();
            Angle tx = usingRight ? getRightLimeLightTx() : getLeftLimeLightTx();

            double targetingLinearVelocityFactor = yAxisPIDController.calculate(
                    aprilTagDistance.in(Units.Inches), ySetPoint.in(Units.Inches));

            double targetingTransversalVelocityFactor = -xAxisPIDController.calculate(
                    tx.in(Units.Degrees), xSetPoint.in(Units.Degrees)) * 0.5;

            swerveDriveDriver.setLongitudinalVelocityFactorSource(() -> targetingLinearVelocityFactor);
            swerveDriveDriver.setTransversalVelocityFactorSource(() -> targetingTransversalVelocityFactor);
        });
    }

}