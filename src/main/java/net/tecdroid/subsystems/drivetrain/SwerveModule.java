// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.tecdroid.subsystems.drivetrain;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import net.tecdroid.util.RotationDirection;
import net.tecdroid.util.CanId;
import net.tecdroid.util.GearRatio;
import net.tecdroid.util.PidfCoefficients;
import net.tecdroid.util.SvagGains;
import net.tecdroid.util.geometry.Wheel;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.constants.UnitConstants.HALF_ROTATION;

public class SwerveModule implements Sendable {

    public record IdentifierConfig(CanId driveId, CanId steerId, CanId absoluteEncoderId) {}
    public record StateConfig(Angle magnetOffset, RotationDirection driveDirection, RotationDirection steerDirection) {}
    public record PhysicalDescription(Translation2d offset, GearRatio driveGearing, GearRatio steerGearing, Wheel wheel) {}
    public record ControlConfig(PidfCoefficients drivePidf, SvagGains driveSvag, PidfCoefficients steerPidf, SvagGains steerSvag) {}
    public record LimitConfig(Current driveCurrentLimit, Time driveRampRate, Current steerCurrentLimit) {}
    public record Config(IdentifierConfig identifiers, StateConfig state, PhysicalDescription physical, ControlConfig control, LimitConfig limits) {}

    private final Config config;

    private final TalonFX driveInterface;

    private final SparkMax                  steerController;
    private final RelativeEncoder           steerEncoder;
    private final SparkClosedLoopController steerClosedLoopController;

    private final CANcoder absoluteEncoder;

    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(Config config) {
        this.config = config;

        this.driveInterface = new TalonFX(config.identifiers.driveId.getId());
        this.configureDriveInterface();

        this.steerController           = new SparkMax(config.identifiers.steerId.getId(), MotorType.kBrushless);
        this.steerEncoder              = steerController.getEncoder();
        this.steerClosedLoopController = steerController.getClosedLoopController();
        this.configureSteerInterface();

        this.absoluteEncoder = new CANcoder(config.identifiers.absoluteEncoderId.getId());
        this.configureAbsoluteEncoderInterface();

    }

    public void setTargetState(SwerveModuleState newState) {
        newState.optimize(new Rotation2d(getSteerShaftAngularPosition()));
        this.targetState = newState;

        final Angle requestedAngle = newState.angle.getMeasure();
        final Angle shaftAngle = config.physical.steerGearing.unapply(requestedAngle);
        steerClosedLoopController.setReference(shaftAngle.in(Rotations), ControlType.kPosition);

        final LinearVelocity requestedVelocity = MetersPerSecond.of(newState.speedMetersPerSecond);
        final AngularVelocity shaftVelocity = config.physical.driveGearing.unapply(config.physical.wheel.linearVelocityToAngularVelocity(requestedVelocity));
        driveInterface.setControl(new VelocityVoltage(shaftVelocity).withSlot(0));
    }

    public void matchSteeringEncoderToAbsoluteEncoder() {
        steerEncoder.setPosition(getAbsoluteSteerShaftPosition().in(Rotations));
    }

    // ///////////////// //
    // Getters + Setters //
    // ///////////////// //

    public Angle getAbsoluteSteerWheelPosition() {
        return absoluteEncoder.getPosition().getValue();
    }

    public Angle getAbsoluteSteerShaftPosition() {
        return config.physical.steerGearing.unapply(getAbsoluteSteerWheelPosition());
    }

    public Angle getDriveShaftPosition() {
        return driveInterface.getPosition().getValue();
    }

    public Angle getDriveWheelAngularDisplacement() {
        return config.physical.driveGearing.apply(getDriveShaftPosition());
    }

    public Distance getDriveWheelLinearDisplacement() {
        return config.physical.wheel.angularDisplacementToLinearDisplacement(getDriveWheelAngularDisplacement());
    }

    public AngularVelocity getDriveShaftAngularVelocity() {
        return driveInterface.getVelocity().getValue();
    }

    public AngularVelocity getDriveWheelAngularVelocity() {
        return config.physical.driveGearing.apply(getDriveShaftAngularVelocity());
    }

    public LinearVelocity getDriveWheelLinearVelocity() {
        return config.physical.wheel.angularVelocityToLinearVelocity(getDriveWheelAngularVelocity());
    }

    public Angle getSteerShaftAngularPosition() {
        return Rotations.of(steerEncoder.getPosition());
    }

    public Angle getSteerWheelAngularPosition() {
        return config.physical.steerGearing.apply(getSteerShaftAngularPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveWheelLinearVelocity(), new Rotation2d(getSteerShaftAngularPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveWheelLinearDisplacement(), new Rotation2d(getSteerShaftAngularPosition()));
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public Translation2d getOffsetFromCenter() {
        return config.physical.offset();
    }

    // ///////////// //
    // Configuration //
    // ///////////// //

    private void configureDriveInterface() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.Audio.withBeepOnBoot(true)
                    .withBeepOnConfig(true)
                    .withAllowMusicDurDisable(true);

        driveConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(config.limits.driveRampRate);

        driveConfig.CurrentLimits.withSupplyCurrentLimit(config.limits.driveCurrentLimit)
                            .withSupplyCurrentLimitEnable(true);

        driveConfig.Slot0.withKP(config.control.drivePidf.getP())
                         .withKI(config.control.drivePidf.getI())
                         .withKD(config.control.drivePidf.getD())
                         .withKS(config.control.driveSvag.getS())
                         .withKV(config.control.driveSvag.getV())
                         .withKA(config.control.driveSvag.getA());

        driveConfig.MotorOutput.withInverted(config.state.driveDirection.toInvertedValue());

        this.driveInterface.setNeutralMode(NeutralModeValue.Coast);
        this.driveInterface.clearStickyFaults();
        this.driveInterface.getConfigurator().apply(driveConfig);
    }

    private void configureSteerInterface() {
        SparkMaxConfig steerConfig = new SparkMaxConfig();

        steerConfig
                .idleMode(IdleMode.kBrake)
                .inverted(config.state.driveDirection.toTrueMeansCounterclockwisePositive())
                .smartCurrentLimit((int)config.limits.steerCurrentLimit.in(Amps));

        steerConfig.closedLoop
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(
                        0.0,
                        config.physical.steerGearing.unapply(HALF_ROTATION.in(Rotations))
                )
                .pidf(
                        config.control.steerPidf.getP(),
                        config.control.steerPidf.getI(),
                        config.control.steerPidf.getD(),
                        config.control.steerPidf.getF()
                );

        this.steerController.clearFaults();
        this.steerController.configure(steerConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    private void configureAbsoluteEncoderInterface() {
        CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();

        absoluteEncoderConfig.MagnetSensor.withSensorDirection(config.state.steerDirection.toSensorDirectionValue())
                                          .withMagnetOffset(config.state.magnetOffset);

        this.absoluteEncoder.clearStickyFaults();
        this.absoluteEncoder.getConfigurator().apply(absoluteEncoderConfig);
    }

    // //// //
    // Data //
    // //// //

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.addDoubleProperty("Position (m)", () -> getDriveWheelLinearDisplacement().in(Meters), (double m) -> {});
        sendableBuilder.addDoubleProperty("Wheel Velocity (m s^-1)", () -> getDriveWheelLinearVelocity().in(MetersPerSecond), (double m) -> {});
        sendableBuilder.addDoubleProperty("Wheel Velocity (rps)", () -> getDriveWheelAngularVelocity().in(RotationsPerSecond), (double m) -> {});
        sendableBuilder.addDoubleProperty("Drive Velocity (rps)", () -> getDriveShaftAngularVelocity().in(RotationsPerSecond), (double m) -> {});
        sendableBuilder.addDoubleProperty("Target Azimuth (deg)", () -> targetState.angle.getDegrees(), (double m) -> {});
        sendableBuilder.addDoubleProperty("Target Velocity (m s^-1)", () -> targetState.speedMetersPerSecond, (double m) -> {});
        sendableBuilder.addDoubleProperty("Azimuth (deg)", () -> getSteerShaftAngularPosition().in(Degrees), (double m) -> {});
        sendableBuilder.addDoubleProperty("Power (%)", driveInterface::get, (double m) -> {});
    }
}
