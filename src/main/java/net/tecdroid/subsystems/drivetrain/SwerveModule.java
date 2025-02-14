// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.tecdroid.subsystems.drivetrain;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.constants.UnitConstants.HALF_ROTATION;
import static net.tecdroid.conventions.MeasurementConventions.MADU;
import static net.tecdroid.conventions.MeasurementConventions.MAVU;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.*;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveUtil.*;

public class SwerveModule implements Sendable {

    private SwerveModuleState targetState = new SwerveModuleState();

    public void setModulePower(Double power) {
        driveTalon.set(power);
    }

    public record Config(Translation2d offset, int driveControllerId, int steerControllerId, int absoluteEncoderId,
                         double magnetOffset, boolean driveInverted) {
    }

    private final Translation2d offset;

    private final TalonFX driveTalon;

    private final SparkMax                  steerController;
    private final RelativeEncoder           steerEncoder;
    private final SparkClosedLoopController steerClosedLoopController;

    private final CANcoder absoluteEncoder;

    public SwerveModule(Config config) {
        this.offset = config.offset();

        this.driveTalon = new TalonFX(config.driveControllerId());
        this.configureDriveInterface(config);

        this.steerController           = new SparkMax(config.steerControllerId(), MotorType.kBrushless);
        this.steerEncoder              = steerController.getEncoder();
        this.steerClosedLoopController = steerController.getClosedLoopController();
        this.configureSteerInterface();

        this.absoluteEncoder = new CANcoder(config.absoluteEncoderId());
        this.configureAbsoluteEncoderInterface(config);

    }

    public Angle getAbsoluteSteerPosition() {
        return absoluteEncoder.getPosition().getValue();
    }

    public Distance getDrivePosition() {
        final double motorRotations = driveTalon.getPosition().getValue().in(Rotations);
        return DRIVE_PCF.times(motorRotations);
    }

    public AngularVelocity getDriveAngularVelocity() {
        return driveTalon.getVelocity().getValue();
    }

    public AngularVelocity getWheelAngularVelocity() {
        return getDriveAngularVelocity().div(DRIVE_GR.getRatio());
    }

    public LinearVelocity getWheelLinearVelocity() {
        return convertFromWheelAngularVelocityToWheelLinearVelocity(getWheelAngularVelocity());
    }

    public LinearAcceleration getWheelAcceleration() {
        final double motorAngularAcceleration = driveTalon.getAcceleration().getValue().in(RotationsPerSecondPerSecond);
        return DRIVE_ACF.times(motorAngularAcceleration);
    }

    public Angle getSteerPosition() {
        return Degrees.of(steerEncoder.getPosition());
    }

    public AngularVelocity getSteerVelocity() {
        return DegreesPerSecond.of(steerEncoder.getVelocity());
    }

    public void setRelativeEncoderPosition(Angle rotation) {
        steerEncoder.setPosition(rotation.in(Degrees));
    }

    public void setAngle(Angle angle) {
        setDesiredState(new SwerveModuleState(0, new Rotation2d(angle)));
    }

    // TESTING
    public void setModuleVoltage(double voltage) {
        driveTalon.setVoltage(voltage);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize(new Rotation2d(getSteerPosition()));
        this.targetState = desiredState;
        //VelocityVoltage(MetersPerSecond.of(desiredState.speedMetersPerSecond))
        final AngularVelocity targetVelocity = convertFromWheelLinearVelocityToDriveAngularVelocity(MetersPerSecond.of(desiredState.speedMetersPerSecond));

        SmartDashboard.putNumber("Requested Velocity rps", targetVelocity.in(RotationsPerSecond));

        steerClosedLoopController.setReference(desiredState.angle.getDegrees(), ControlType.kPosition);

        VelocityVoltage request = new VelocityVoltage(targetVelocity).withSlot(0);;
        driveTalon.setControl(request);

    }

    public void seed() {
        // Either change getAbsoluteSteerPosition to Rotation2d, or change setRelativeEncoderPosition to take Angle (done)
        setRelativeEncoderPosition(getAbsoluteSteerPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelLinearVelocity(), new Rotation2d(getSteerPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerPosition()));
    }

    public Translation2d getOffset() {
        return offset;
    }

    private void configureDriveInterface(Config cfg) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.withBeepOnBoot(true)
                    .withBeepOnConfig(true)
                    .withAllowMusicDurDisable(true);

        config.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(DRIVE_RR);

        config.CurrentLimits.withSupplyCurrentLimit(DRIVE_CR)
                            .withSupplyCurrentLimitEnable(true);

        config.Slot0.withKP(Pidf.DRIVE.p())
                    .withKI(Pidf.DRIVE.i())
                    .withKD(Pidf.DRIVE.d())
                    .withKS(Sva.DRIVE.s())
                    .withKV(Sva.DRIVE.v())
                    .withKA(Sva.DRIVE.a());

        config.MotorOutput.withInverted(cfg.driveInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);

        this.driveTalon.setNeutralMode(NeutralModeValue.Coast);
        this.driveTalon.clearStickyFaults();
        this.driveTalon.getConfigurator().apply(config);
    }

    private void configureSteerInterface() {
        SparkMaxConfig steerConfig = new SparkMaxConfig();

        steerConfig
                .idleMode(IdleMode.kBrake)
                .inverted(true);

        steerConfig.encoder
                .positionConversionFactor(STEER_PCF.in(MADU))
                .velocityConversionFactor(STEER_VCF.in(MAVU));

        steerConfig.closedLoop
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0.0, HALF_ROTATION.in(Degrees))
                .pidf(
                        Pidf.STEER.p(),
                        Pidf.STEER.i(),
                        Pidf.STEER.d(),
                        Pidf.STEER.f()
                );

        this.steerController.clearFaults();
        this.steerController.configure(steerConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    private void configureAbsoluteEncoderInterface(Config config) {
        CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();

        absoluteEncoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                                          .withMagnetOffset(config.magnetOffset());

        this.absoluteEncoder.clearStickyFaults();
        this.absoluteEncoder.getConfigurator().apply(absoluteEncoderConfig);
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.addDoubleProperty("Position (m)", () -> getDrivePosition().in(Meters), (double m) -> {});
        sendableBuilder.addDoubleProperty("Wheel Velocity (m s^-1)", () -> getWheelLinearVelocity().in(MetersPerSecond), (double m) -> {});
        sendableBuilder.addDoubleProperty("Wheel Velocity (rps)", () -> getWheelAngularVelocity().in(RotationsPerSecond), (double m) -> {});
        sendableBuilder.addDoubleProperty("Drive Velocity (rps)", () -> getDriveAngularVelocity().in(RotationsPerSecond), (double m) -> {});
        sendableBuilder.addDoubleProperty("Target Azimuth (deg)", () -> targetState.angle.getDegrees(), (double m) -> {});
        sendableBuilder.addDoubleProperty("Target Velocity (m s^-1)", () -> targetState.speedMetersPerSecond, (double m) -> {});
        sendableBuilder.addDoubleProperty("Azimuth (deg)", () -> getSteerPosition().in(Degrees), (double m) -> {});
        sendableBuilder.addDoubleProperty("Power (%)", driveTalon::get, (double m) -> {});
    }

    public int getModuleDigit() {
        return steerController.getDeviceId() / 10;
    }

    public void linkModule(SwerveModule module) {
        driveTalon.setControl(new Follower(module.driveTalon.getDeviceID(), false));
    }

}
