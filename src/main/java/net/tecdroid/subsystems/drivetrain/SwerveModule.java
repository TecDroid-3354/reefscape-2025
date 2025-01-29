// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.tecdroid.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.Degrees;
import static net.tecdroid.constants.UnitConstants.QUARTER_ROTATION;
import static net.tecdroid.conventions.MeasurementConventions.*;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.*;

public class SwerveModule {

    public record Config(Translation2d offset, int driveControllerId, int steerControllerId, int absoluteEncoderId,
                         double magnetOffset) {
    }

    private final Translation2d offset;

    private final SparkMax                  driveController;
    private final RelativeEncoder           driveEncoder;
    private final SparkClosedLoopController driveClosedLoopController;

    private final SparkMax                  steerController;
    private final RelativeEncoder           steerEncoder;
    private final SparkClosedLoopController steerClosedLoopController;

    private final CANcoder absoluteEncoder;

    public SwerveModule(Config config) {
        this.offset = config.offset();

        // Setting the controller, encoder and closed loop controller for the speed motor
        // Speed Controller, Sensor & PIDController
        driveController = new SparkMax(config.driveControllerId(), MotorType.kBrushless);
        driveController.clearFaults();

        // Spark configuration (Idle mode, PIDF, Continous Input)
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        driveConfig
                .idleMode(IdleMode.kBrake)
                .closedLoopRampRate(0.25);

        driveConfig.encoder
                .positionConversionFactor(DRIVE_ENCODER_PCF.in(MEASUREMENT_DISTANCE_UNIT))
                .velocityConversionFactor(DRIVE_ENCODER_VCF.in(MEASUREMENT_LINEAR_VELOCITY_UNIT));

        driveConfig.closedLoop
                .pidf(
                        Pidf.DRIVE.p(),
                        Pidf.DRIVE.i(),
                        Pidf.DRIVE.d(),
                        Pidf.DRIVE.f()
                );

        // Apply the configuration
        driveController.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        this.driveEncoder              = driveController.getEncoder();
        this.driveClosedLoopController = driveController.getClosedLoopController();

        // ! Setting rotation spark basic configuration, including PID and idleMode & clear sticky faults
        // Rotation Controller, Sensor & PIDController
        steerController = new SparkMax(config.steerControllerId(), MotorType.kBrushless);
        steerController.clearFaults();

        SparkMaxConfig steerConfig = new SparkMaxConfig();

        steerConfig
                .idleMode(IdleMode.kBrake)
                .inverted(true);

        steerConfig.encoder
                .positionConversionFactor(STEER_ENCODER_PCF.in(MEASUREMENT_ANGLE_UNIT))
                .velocityConversionFactor(STEER_ENCODER_VCF.in(MEASUREMENT_ANGULAR_VELOCITY_UNIT));

        steerConfig.closedLoop
                // Position wrapping —— Range resets when it surpasses either parameter
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0.0, QUARTER_ROTATION.in(Degrees))
                .pidf(
                        Pidf.STEER.p(),
                        Pidf.STEER.i(),
                        Pidf.STEER.d(),
                        Pidf.STEER.f()
                );

        // Setting the configuration up for rotation spark & clear sticky faults
        steerController.configure(steerConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        this.steerEncoder              = steerController.getEncoder();
        this.steerClosedLoopController = steerController.getClosedLoopController();

        CANcoderConfiguration absoluteEncoderConfig              = new CANcoderConfiguration();
        MagnetSensorConfigs   absoluteEncoderMagnetSensorConfigs = new MagnetSensorConfigs();
        absoluteEncoder = new CANcoder(config.absoluteEncoderId());
        absoluteEncoder.clearStickyFaults();

        absoluteEncoderMagnetSensorConfigs
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(config.magnetOffset());

        absoluteEncoderConfig.withMagnetSensor(absoluteEncoderMagnetSensorConfigs);

        absoluteEncoder.getConfigurator()
                       .apply(absoluteEncoderConfig);

    }

    public Rotation2d getAbsoluteEncoderPosition() {
        return Rotation2d.fromRotations(absoluteEncoder.getPosition()
                                                       .getValueAsDouble());
    }

    public Distance getDrivePosition() {
        return MEASUREMENT_DISTANCE_UNIT.of(driveEncoder.getPosition());
    }

    public LinearVelocity getDriveVelocity() {
        return MEASUREMENT_LINEAR_VELOCITY_UNIT.of(driveEncoder.getVelocity());
    }

    public Angle getSteerPosition() {
        return MEASUREMENT_ANGLE_UNIT.of(steerEncoder.getPosition());
    }

    public AngularVelocity getSteerVelocity() {
        return MEASUREMENT_ANGULAR_VELOCITY_UNIT.of(steerEncoder.getVelocity());
    }

    public void setRelativeEncoderPosition(Rotation2d rotation) {
        steerEncoder.setPosition(rotation.getDegrees());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize((Rotation2d) getSteerPosition());
        driveClosedLoopController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        steerClosedLoopController.setReference(desiredState.angle.getDegrees(), ControlType.kPosition);
    }

    public void seed() {
        setRelativeEncoderPosition(getAbsoluteEncoderPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), (Rotation2d) getSteerPosition());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), (Rotation2d) getSteerPosition());
    }

    public Translation2d getOffset() {
        return offset;
    }
}
