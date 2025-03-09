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

    public Distance getLeftLimeLightDistance() {
        return leftLimeLight.getDistance();
    }

    public Distance getRightLimeLightDistance() {
        return rightLimeLight.getDistance();
    }

    public Angle getLeftLimeLightTx() {
        return leftLimeLight.getTx();
    }

    public Angle getRightLimeLightTx() {
        return rightLimeLight.getTx();
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

    public Command alignYAxisToAprilTagDetection(SwerveDriveDriver swerveDriveDriver, Distance setpoint,
                                                 Boolean usingRight) {
        return Commands.run(() -> {

            Distance detectionDistance = usingRight ? getRightLimeLightDistance() : getLeftLimeLightDistance();

            double targetingLinearVelocityFactor = yAxisPIDController.calculate(
                    detectionDistance.in(Units.Inches), setpoint.in(Units.Inches));

            swerveDriveDriver.setLongitudinalVelocityFactorSource(() -> targetingLinearVelocityFactor);
        });
    }

}