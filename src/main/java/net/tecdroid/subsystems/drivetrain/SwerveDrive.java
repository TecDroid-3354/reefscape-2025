package net.tecdroid.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import kotlin.Pair;
import net.tecdroid.util.NumericId;

import java.util.Arrays;
import java.util.List;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule[] modules;

    private final Pigeon2 gyro;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry   odometry;

    private final Config config;

    public SwerveDrive(Config config) {
        this.config = config;

        this.modules =
            config.moduleConfig.moduleConfigs.stream().map((pair) -> new SwerveModule(pair.getFirst())).toArray(SwerveModule[]::new);

        this.gyro = new Pigeon2(config.identifierConfig.imuId().getId());
        this.configureImuInterface();

        this.kinematics =
            new SwerveDriveKinematics(config.moduleConfig.moduleConfigs.stream().map(Pair::getSecond).toArray(Translation2d[]::new));

        this.odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(getHeading()), getModulePositions());
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

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

    // //////// //
    // Odometry //
    // //////// //

    public void updateOdometry() {
        odometry.update(new Rotation2d(getHeading()), getModulePositions());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d newPose) {
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), newPose);
    }

    // ///// //
    // State //
    // ///// //

    public Angle getHeading() {
        return gyro.getYaw().getValue();
    }

    // ///////////////// //
    // Getters + Setters //
    // ///////////////// //

    public void setHeading(Angle angle) {
        gyro.setYaw(angle);
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
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

    public record ModuleConfig(List<Pair<SwerveModule.Config, Translation2d>> moduleConfigs) {
    }

    public record DeviceIdentifiers(NumericId imuId) {
    }

    public record Config(ModuleConfig moduleConfig, DeviceIdentifiers identifierConfig) {
    }
}
