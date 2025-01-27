// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.tecdroid.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;

import static net.tecdroid.conventions.MeasurementConventions.MEASUREMENT_ANGLE_UNIT;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.Pidf;

public class SwerveDrive extends SubsystemBase {
    public record Config(SwerveModule.Config[] moduleConfigs, int gyroId) {
    }

    private final SwerveModule[] modules;
    private final Pigeon2        gyro;
    private final PIDController  turnPidController;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry   odometry;

    private Pose2d pose;

    public SwerveDrive(Config config) {
        this.modules = Arrays.stream(config.moduleConfigs())
                             .map(SwerveModule::new)
                             .toArray(SwerveModule[]::new);

        this.gyro = new Pigeon2(config.gyroId());
        gyro.clearStickyFaults();
        gyro.setYaw(0.0);

        this.turnPidController = new PIDController(Pidf.ANGLE.p(), Pidf.ANGLE.i(), Pidf.ANGLE.d());
        turnPidController.enableContinuousInput(-180, 180);

        this.kinematics = new SwerveDriveKinematics(Arrays.stream(modules)
                                                          .map(SwerveModule::getOffset)
                                                          .toArray(Translation2d[]::new));

        this.odometry = new SwerveDriveOdometry(kinematics, (Rotation2d) getHeading(), getModulePositions());
    }

    @Override
    public void periodic() {
        updateOdometry();
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
                turnPidController.calculate(getHeading().in(MEASUREMENT_ANGLE_UNIT), angleTarget.getDegrees()),
                -1, 1
        ));

        double angularVelocity = SwerveDriveUtil.denormalizeAngularVelocity(rotationNormalizedPidOutput);

        chassisSpeeds.omegaRadiansPerSecond = Math.toRadians(angularVelocity);
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

    public void setPose(Pose2d newPose) {
        odometry.resetPosition(
                gyro.getRotation2d(),
                getModulePositions(),
                newPose
        );
    }

    public void updateOdometry() {
        odometry.update((Rotation2d) getHeading(), getModulePositions());
        pose = odometry.getPoseMeters();
    }

    public Angle getHeading() {
        return gyro.getYaw()
                   .getValue();
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
