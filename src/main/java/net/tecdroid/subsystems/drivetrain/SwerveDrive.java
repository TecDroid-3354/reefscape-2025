// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.tecdroid.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.tecdroid.util.CanId;

import java.util.Arrays;

public class SwerveDrive extends SubsystemBase {
    public record ModuleConfig(SwerveModule.Config[] moduleConfigs) {}

    public record IdentifierConfig(CanId imuId) {}
    public record Config(ModuleConfig moduleConfig, IdentifierConfig identifierConfig) {}

    private final SwerveModule[] modules;
    private final Pigeon2        gyro;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry   odometry;

    private Pose2d pose;

    private final Config config;

    public SwerveDrive(Config config) {
        this.config = config;

        this.modules = Arrays.stream(config.moduleConfig.moduleConfigs())
                             .map(SwerveModule::new)
                             .toArray(SwerveModule[]::new);

        this.gyro = new Pigeon2(config.identifierConfig.imuId().getId());
        this.configureImuInterface();

        this.kinematics = new SwerveDriveKinematics(
                Arrays.stream(modules)
                      .map(SwerveModule::getOffsetFromCenter)
                      .toArray(Translation2d[]::new)
        );

        this.odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(getHeading()), getModulePositions());
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    // ///// //
    // Drive //
    // ///// //

    public void setModuleTargetStates(SwerveModuleState... states) {
        assert (states.length == modules.length);

        for (int i = 0; i < states.length; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleTargetStates(desiredStates);
    }

    public void matchModuleSteeringEncodersToAbsoluteEncoders() {
        for (SwerveModule module : modules) {
            module.matchSteeringEncoderToAbsoluteEncoder();
        }
    }

    // ///// //
    // State //
    // ///// //

    public void updateOdometry() {
        odometry.update(new Rotation2d(getHeading()), getModulePositions());
        pose = odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(getHeading()), getModulePositions(), pose);
    }

    // ///////////////// //
    // Getters + Setters //
    // ///////////////// //

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d newPose) {
        odometry.resetPosition(
                gyro.getRotation2d(),
                getModulePositions(),
                newPose
        );
    }
    public Angle getHeading() {
        return gyro.getYaw()
                   .getValue();
    }

    public void setHeading(Angle angle) {
        gyro.setYaw(angle);
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

    // ///////////// //
    // Configuration //
    // ///////////// //

    public void configureImuInterface() {
        Pigeon2Configuration imuConfiguration = new Pigeon2Configuration();

        gyro.setYaw(0.0);
        gyro.clearStickyFaults();
        gyro.getConfigurator().apply(imuConfiguration);

    }
}
