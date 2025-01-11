package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
import frc.robot.Robot;
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

    // Path planner robot config
    RobotConfig config;

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

        // !!!! Path Planner !!!!
        
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setChassisSpeed(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.1, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );

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

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), 
        new SwerveModulePosition[] {
            frontRight.getPosition(),
            frontLeft.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
          }, 
          pose);
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
        ChassisSpeeds.discretize(chassisSpeeds, Robot.kDefaultPeriod); // Mili seconds of the roborio
        SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(desiredStates);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                                                               frontLeft.getState(),
                                                               backRight.getState(),
                                                               backLeft.getState());
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
