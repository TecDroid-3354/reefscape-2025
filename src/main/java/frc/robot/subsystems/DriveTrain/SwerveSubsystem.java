package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    //private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final Pigeon2 gyro = new Pigeon2(SwerveConstants.DriveConstants.kPigeon2Port);

    // Rotation PID
    PIDController rotationPID;

    SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, getRotation2d(),
        new SwerveModulePosition[] {
            frontRight.getPosition(),
            frontLeft.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
          }, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        rotationPID = new PIDController(SwerveConstants.RotateConstants.kP, SwerveConstants.RotateConstants.kI, SwerveConstants.RotateConstants.kD);
        ShuffleboardData();
    }

    public void zeroHeading() {
        gyro.reset();
    }
    
    public double getGyroYaw() {
        return gyro.getYaw().getValueAsDouble() * (SwerveConstants.DriveConstants.gyroReversed ? -1.0 : 1.0);
    }

    public double getHeading() {
        return Math.IEEEremainder(getGyroYaw(), 360);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public double getRobotYawInRad() {
        return gyro.getYaw().getValueAsDouble() * Math.PI/180;
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            frontRight.getPosition(),
            frontLeft.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
          },pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
            frontRight.getPosition(),
            frontLeft.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
          });
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void ShuffleboardData() {
        // Shuffleboard
        ShuffleboardTab swerveTab = Shuffleboard.getTab("CanCoders reads");
        swerveTab.addDouble("FrontRight CanCoderRead", () -> {return frontRight.getAbsoluteEncoderRad();});
        swerveTab.addDouble("FrontLeft CanCoderRead", () -> {return frontLeft.getAbsoluteEncoderRad();});
        swerveTab.addDouble("BackRight CanCoderRead", () -> {return backRight.getAbsoluteEncoderRad();});
        swerveTab.addDouble("BackLeft CanCoderRead", () -> {return backLeft.getAbsoluteEncoderRad();});

        swerveTab.addDouble("FrontRight TurningPosition", () -> {return frontRight.getTurningPosition();});
        swerveTab.addDouble("FrontLeft TurningPosition", () -> {return frontLeft.getTurningPosition();});
        swerveTab.addDouble("BackRight TurningPosition", () -> {return backRight.getTurningPosition();});
        swerveTab.addDouble("BackLeft TurningPosition", () -> {return backLeft.getTurningPosition();});
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds (desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

    public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
        //ChassisSpeeds.discretize(chassisSpeeds, ); Mili seconds of the roborio
        SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(desiredStates);
    }

    public void setFromFieldOrintedChassisSpeed(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getRotation2d());
        setChassisSpeed(chassisSpeeds);
    }

    // Autonomus while align to a apriltag detection and to rotate to a degree setpoint

    public ChassisSpeeds chassisSpeedAdition(ChassisSpeeds... chassisSpeeds) {
        double vxMetersPerSecond = 0;
        double vyMetersPerSecond = 0;
        double omegaRadiansPerSecond = 0;

        for (ChassisSpeeds chassisSpeed : chassisSpeeds) {
            vxMetersPerSecond += chassisSpeed.vxMetersPerSecond;
            vyMetersPerSecond += chassisSpeed.vyMetersPerSecond;
            omegaRadiansPerSecond += chassisSpeed.omegaRadiansPerSecond;
        }
        return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }
    
}
