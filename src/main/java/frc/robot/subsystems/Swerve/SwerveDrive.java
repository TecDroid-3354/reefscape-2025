// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants.Modules.SparkIDs;
import frc.robot.Constants.SwerveConstants.Modules.ModulesRelativeEncodersIDs;
import frc.robot.Constants.SwerveConstants.AbsoluteEncoderID;
import frc.robot.util.SwerveDriveUtil;

import static frc.robot.Constants.SwerveConstants.Kinematics.TRACK_WIDTH;
import static frc.robot.Constants.SwerveConstants.Kinematics.WHEEL_BASE;
import static frc.robot.Constants.SwerveConstants.Modules.AbsoluteEncoders.*;

/** Add your docs here. */
public class SwerveDrive extends SubsystemBase {

    /*----- INITIALIZE THE FOUR SWERVE MODULES -----*/
    private final SwerveModule frontLeftModule = new SwerveModule(
        SparkIDs.SPEED_FRONT_LEFT_ID, SparkIDs.ROTATION_FRONT_LEFT_ID, ModulesRelativeEncodersIDs.FRONT_LEFT_ID, FRONT_LEFT_OFFSET);
    
    private final SwerveModule frontRightModule = new SwerveModule(
        SparkIDs.SPEED_FRONT_RIGHT_ID, SparkIDs.ROTATION_FRONT_RIGHT_ID, ModulesRelativeEncodersIDs.FRONT_RIGHT_ID, FRONT_RIGHT_OFFSET);

    private final SwerveModule backLeftModule = new SwerveModule(
        SparkIDs.SPEED_BACK_LEFT_ID, SparkIDs.ROTATION_BACK_LEFT_ID, ModulesRelativeEncodersIDs.BACK_LEFT_ID, BACK_LEFT_OFFSET);

    private final SwerveModule backRightModule = new SwerveModule(
        SparkIDs.SPEED_BACK_RIGHT_ID, SparkIDs.ROTATION_BACK_RIGHT_ID, ModulesRelativeEncodersIDs.BACK_RIGHT_ID, BACK_RIGHT_OFFSET);
    
    /*----- INITIALIZE THE SWERVE GYRO -----*/
    private final Pigeon2 gyro = new Pigeon2(AbsoluteEncoderID.ABSOLUTE_ENCODER_ID);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    private final PIDController rotationPidController = new PIDController(
        0.0055, 
        0.0, 
        0.0);

    // Creating my odometry object from the kinematics object and the initial wheel positions.
    // Here, our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing the opposing alliance wall.
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      kinematics, gyro.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
      }, new Pose2d(5.0, 13.5, new Rotation2d())
    );
    
    private Pose2d robotPose;

    public SwerveDrive() {

        setModuleStates(
            new SwerveModuleState(0, new Rotation2d()), 
            new SwerveModuleState(0, new Rotation2d()), 
            new SwerveModuleState(0, new Rotation2d()), 
            new SwerveModuleState(0, new Rotation2d())
        );

        gyro.setYaw(0.0);
        gyro.clearStickyFaults();

        shuffleBoardData();

        rotationPidController.enableContinuousInput(-180, 180);
    }

    public void shuffleBoardData() {
        ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
        tab.addDouble("Heading", () -> getGyroPosition().getDegrees());
    }

    @Override
    public void periodic() {
        // Update the pose
        robotPose = odometry.update(gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(), frontRightModule.getPosition(),
                backLeftModule.getPosition(), backRightModule.getPosition()
            }
        );

        SmartDashboard.putNumber("aaaaaaa", getHeading().getDegrees());
    }

    public void setModuleStates(SwerveModuleState frontLeftState, SwerveModuleState frontRightState, 
                                    SwerveModuleState backLeftState, SwerveModuleState backRightState) {
        /*Sends the desired state to each module*/
        frontLeftModule.setDesiredState(frontLeftState);
        frontRightModule.setDesiredState(frontRightState);
        backLeftModule.setDesiredState(backLeftState);
        backRightModule.setDesiredState(backRightState);
    }
    
    public void drive(ChassisSpeeds chassisSpeeds, Rotation2d angleTarget) {
        /*Receives the chassisSpeeds, transform them into SwerveModuleStates and apply them to each module*/        
        double rotationNormalizedPidOutput = -(MathUtil.clamp(
            rotationPidController.calculate(getGyroPosition().getDegrees(), angleTarget.getDegrees()),
            -1, 1
        ));

        double angularVelocity = SwerveDriveUtil.denormalizeAngularVelocity(rotationNormalizedPidOutput);

        // Convert angularVelocity to radians.
        chassisSpeeds.omegaRadiansPerSecond = Math.toRadians(angularVelocity);
        
        // ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(chassisSpeeds, Robot.kDefaultPeriod);
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // [0] -> Front_Left  [1] -> Front_Right    [2] -> Back_Left   [3] -> Back_Right
        setModuleStates(desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3]);
    }

    public void seedEncoders() {
        /*Set all absolute encoders to the gyro's reading*/
        frontLeftModule.seed(); 
        frontRightModule.seed(); 
        backLeftModule.seed(); 
        backRightModule.seed(); 
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public void resetOdometryPosition(Pose2d newRobotPose) {
        /*Resets the robot position given a new field-relative Pose2d*/
        odometry.resetPosition(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(), frontRightModule.getPosition(),
                backLeftModule.getPosition(), backRightModule.getPosition()
            },
            newRobotPose
        );
    }

    public Rotation2d getGyroPosition() {
        /*Returns the gyro's reading as a Rotation2d object*/
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public Rotation2d getHeading() {
        return robotPose.getRotation();
    }
}
