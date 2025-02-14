// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.tecdroid.subsystems.drivetrain;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Arrays;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static net.tecdroid.conventions.MeasurementConventions.MADU;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.Pidf;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveUtil.denormalizeAngularVelocity;

public class SwerveDrive extends SubsystemBase {
    public record Config(SwerveModule.Config[] moduleConfigs, int gyroId) {
    }

    private final VoltageOut voltage_request = new VoltageOut(0.0);

    private final SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(3),
                            null,
                            (state) -> SignalLogger.writeString("state", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            (volts) -> setVoltage(voltage_request.withOutput(volts.in(Volts))),
                            null,
                            this
                    )
            );

    private final SwerveModule[] modules;
    private final Pigeon2        gyro;
    private final PIDController  turnPidController;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry   odometry;

    private Pose2d pose;

    SwerveDriveTab swerveDriveTab;
    public SwerveDrive(Config config) {
        swerveDriveTab = new SwerveDriveTab();
        this.modules = Arrays.stream(config.moduleConfigs())
                             .map(SwerveModule::new)
                             .toArray(SwerveModule[]::new);
        swerveDriveTab.publishDrive(this);
        swerveDriveTab.publishModules(modules);

        this.gyro = new Pigeon2(config.gyroId());
        gyro.clearStickyFaults();
        gyro.setYaw(0.0);

        this.turnPidController = new PIDController(Pidf.ANGLE.p(), Pidf.ANGLE.i(), Pidf.ANGLE.d());
        turnPidController.enableContinuousInput(-180, 180);

        this.kinematics = new SwerveDriveKinematics(Arrays.stream(modules)
                                                          .map(SwerveModule::getOffset)
                                                          .toArray(Translation2d[]::new));

        this.odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(getHeading()), getModulePositions());
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    // SYSID system
    public Command quasistaticRoutine(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command dynamicRoutine(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    // TESTING
    public void setVoltage(VoltageOut voltage_request) {
        for (SwerveModule module : modules) {
            module.setModuleVoltage(voltage_request.Output);
        }
    }

    public void setPower(Double power) {
        for (SwerveModule module : modules) {
            module.setModulePower(power);
        }
    }

    public Command setModuleAngles(Angle angle) {
        return Commands.runOnce(() -> {for (SwerveModule module : modules) {
            module.setAngle(angle);
        }}, this);
    }

    public void setDesiredModuleStates(SwerveModuleState... states) {
        assert (states.length == modules.length);
        for (int i = 0; i < states.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        setDesiredModuleStates(desiredStates);
    }

    public void drive(ChassisSpeeds chassisSpeeds, Rotation2d angleTarget) {
        double rotationNormalizedPidOutput = -(MathUtil.clamp(
                turnPidController.calculate(getHeading().in(MADU), angleTarget.getDegrees()),
                -1, 1
        ));

        AngularVelocity angularVelocity = denormalizeAngularVelocity(rotationNormalizedPidOutput);

        chassisSpeeds.omegaRadiansPerSecond = angularVelocity.in(RadiansPerSecond);
        drive(chassisSpeeds);
    }

    public void seedEncoders() {
        for (SwerveModule module : modules) {
            module.seed();
        }
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public void linkModules() {
        SwerveModule firstModule = modules[0];
        for (int i = 1; i < modules.length; i++) {
            SwerveModule module = modules[i];
            module.linkModule(firstModule);
        }
    }

    public void setPose(Pose2d newPose) {
        odometry.resetPosition(
                gyro.getRotation2d(),
                getModulePositions(),
                newPose
        );
    }

    public void updateOdometry() {
        odometry.update(new Rotation2d(getHeading()), getModulePositions());
        pose = odometry.getPoseMeters();
    }

    public Angle getHeading() {
        return gyro.getYaw()
                   .getValue();
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules)
                     .map(SwerveModule::getState)
                     .toArray(SwerveModuleState[]::new);
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules)
                     .map(SwerveModule::getPosition)
                     .toArray(SwerveModulePosition[]::new);
    }
}
