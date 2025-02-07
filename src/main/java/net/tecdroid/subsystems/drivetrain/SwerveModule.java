// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.tecdroid.subsystems.drivetrain;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.constants.UnitConstants.QUARTER_ROTATION;
import static net.tecdroid.conventions.MeasurementConventions.*;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.*;

public class SwerveModule implements Sendable {

    public record Config(Translation2d offset, int driveControllerId, int steerControllerId, int absoluteEncoderId,
                         double magnetOffset) {
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
        this.configureDriveInterface();

        this.steerController = new SparkMax(config.steerControllerId(), MotorType.kBrushless);
        this.steerEncoder              = steerController.getEncoder();
        this.steerClosedLoopController = steerController.getClosedLoopController();
        this.configureSteerInterface();

        absoluteEncoder = new CANcoder(config.absoluteEncoderId());
        this.configureAbsoluteEncoderInterface(config);

    }

    public Angle getAbsoluteSteerPosition() {
        return absoluteEncoder.getPosition().getValue();
    }

    public Distance getDrivePosition() {
        final double motorRotations = driveTalon.getPosition().getValue().in(Rotations);
        return DRIVE_PCF.times(motorRotations);
    }

    public LinearVelocity getDriveVelocity() {
        final double motorAngularVelocity = driveTalon.getVelocity().getValue().in(RotationsPerSecond);
        return DRIVE_VCF.times(motorAngularVelocity);
    }

    public LinearAcceleration getDriveAcceleration() {
        final double motorAngularAcceleration = driveTalon.getAcceleration().getValue().in(RotationsPerSecondPerSecond);
        return DRIVE_ACF.times(motorAngularAcceleration);
    }

    public Angle getSteerPosition() {
        return MADU.of(steerEncoder.getPosition());
    }

    public AngularVelocity getSteerVelocity() {
        return MAVU.of(steerEncoder.getVelocity());
    }

    public void setRelativeEncoderPosition(Angle rotation) {
        steerEncoder.setPosition(rotation.in(MADU));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize((Rotation2d) getSteerPosition());
        //VelocityVoltage(MetersPerSecond.of(desiredState.speedMetersPerSecond))
        final VelocityVoltage targetVelocity = new VelocityVoltage(desiredState.speedMetersPerSecond).withSlot(0);
        driveTalon.setNeutralMode(NeutralModeValue.Coast);
        steerClosedLoopController.setReference(desiredState.angle.getDegrees(), ControlType.kPosition);
    }

    public void seed() {
        // Either change getAbsoluteSteerPosition to Rotation2d, or change setRelativeEncoderPosition to take Angle (done)
        setRelativeEncoderPosition(getAbsoluteSteerPosition());
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

    private void configureDriveInterface() {
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
                .positionWrappingInputRange(0.0, QUARTER_ROTATION.in(Degrees))
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
        sendableBuilder.addDoubleProperty("Acceleration (m s^-2)", () -> getDriveAcceleration().in(MLAU), (double m) -> {});
        sendableBuilder.addDoubleProperty("Position (m)", () -> getDrivePosition().in(MLDU), (double m) -> {});
        sendableBuilder.addDoubleProperty("Velocity (m s^-1)", () -> getDriveVelocity().in(MLVU), (double m) -> {});
        sendableBuilder.addDoubleProperty("Azimuth (deg)", () -> getSteerPosition().in(MADU), (double m) -> {});
    }

}
