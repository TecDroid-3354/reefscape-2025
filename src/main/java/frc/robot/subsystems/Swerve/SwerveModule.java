// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.SwerveConstants.Modules.UnitsConversion;
import frc.robot.Constants.SwerveConstants.Modules.SparkPID;

/** Add your docs here. */
public class SwerveModule {
    // Speed Controller, Sensor & PIDController
    private final SparkMax speedMotorController;
    private final RelativeEncoder speedMotorEncoder;
    private final SparkClosedLoopController speedClosedLoopController;

    // Rotation Controller, Sensor & PIDController
    private final SparkMax rotationMotorController;
    private final RelativeEncoder rotationMotorEncoder;
    private final SparkClosedLoopController rotationClosedLoopController;

    // Swerve absolute encoder
    private final CANcoder absoluteEncoder;

    public SwerveModule(int speedMotorID, int rotationMotorID, int absoluteEncoderID, double magnetOffset) {
        // Setting the controller, encoder and closed loop controller for the speed motor
        speedMotorController = new SparkMax(speedMotorID, MotorType.kBrushless);
        speedMotorEncoder = speedMotorController.getEncoder();
        speedClosedLoopController = speedMotorController.getClosedLoopController();

        // Spark configuration (Idle mode, PIDF, Continous Input)
        SparkMaxConfig speedMotorConfig = new SparkMaxConfig();

        speedMotorConfig
            .idleMode(IdleMode.kBrake)
            .closedLoopRampRate(0.25);

        speedMotorConfig.encoder
            .positionConversionFactor(UnitsConversion.SPEED_ENCODER_ROTATIONS_TO_METERS)
            .velocityConversionFactor(UnitsConversion.SPEED_ENCODER_RPM_TO_METERS_PER_SECOND);

        speedMotorConfig.closedLoop
            .pidf(
                SparkPID.Speed.kP, 
                SparkPID.Speed.kI, 
                SparkPID.Speed.kD, 
                SparkPID.Speed.kFF
            );
        
        // Apply the configuration
        speedMotorController.configure(speedMotorConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        speedMotorController.clearFaults();


        // ! Setting rotation spark basic configuration, including PID and idleMode & clear sticky faults
        rotationMotorController = new SparkMax(rotationMotorID, MotorType.kBrushless);
        rotationMotorEncoder = rotationMotorController.getEncoder();
        rotationClosedLoopController = rotationMotorController.getClosedLoopController();

        SparkMaxConfig rotationMotorConfig = new SparkMaxConfig();
        
        rotationMotorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        
        rotationMotorConfig.encoder
            .positionConversionFactor(UnitsConversion.ROTATION_ENCODER_ROTATIONS_TO_DEGREES);

        rotationMotorConfig.closedLoop
            // Position wrapping —— Range resets when it surpasses either parameter 
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-90, 90)
            .pid(
                SparkPID.Rotation.kP,
                SparkPID.Rotation.kI,
                SparkPID.Rotation.kD
            );
        
        // Setting the configuration up for rotation spark & clear sticky faults
        rotationMotorController.configure(rotationMotorConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rotationMotorController.clearFaults();

        CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs absoluteEncoderMagnetSensorConfigs = new MagnetSensorConfigs();
        absoluteEncoder = new CANcoder(absoluteEncoderID);

        absoluteEncoderMagnetSensorConfigs
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(magnetOffset);
        absoluteEncoderConfig.withMagnetSensor(absoluteEncoderMagnetSensorConfigs);
        absoluteEncoder.getConfigurator().apply(absoluteEncoderConfig);

        ShuffleBoardData(absoluteEncoderID);
    }

    public void ShuffleBoardData(int moduleNumber) {
        ShuffleboardTab moduleData = Shuffleboard.getTab("Swerve");
        // moduleData.addDouble("(" + moduleNumber + ") Drive Velocity", this::getSpeedEncoderVelocity);
        // moduleData.addDouble("(" + moduleNumber + ") Drive Position", this::getSpeedEncoderPosition);
        moduleData.addDouble("(" + moduleNumber + ") Steer Position", () -> getRotationEncoderPosition().getDegrees());
        moduleData.addDouble("(" + moduleNumber + ") Absol Position", () -> getAbsoluteEncoderPosition().getDegrees());
    }

    public Rotation2d getAbsoluteEncoderPosition() {
        /*Returns the relative encoder's reading*/
        return Rotation2d.fromRotations(absoluteEncoder.getPosition().getValueAsDouble());
    }

    public double getSpeedEncoderVelocity() {
        /*Returns the encoder's reading in meters per second*/
        return speedMotorEncoder.getVelocity();
    }

    public double getSpeedEncoderPosition() {
        /*Returns the encoder's reading in meters*/
        return speedMotorEncoder.getPosition();
    }
    
    public Rotation2d getRotationEncoderPosition() {
        /*Returns the encoder's reading*/
        return Rotation2d.fromDegrees(rotationMotorEncoder.getPosition());
    }

    public double getRotationEncoderVelocity() {
        return rotationMotorEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getSpeedEncoderPosition(), getRotationEncoderPosition());
    }

    public void setRelativeEncoderPosition(Rotation2d rotation) {
        /*Used to match the relative and absolute encoder readings*/
        rotationMotorEncoder.setPosition(rotation.getDegrees());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        /*Uses the spark's internal PID to set the desired speed and position*/
        desiredState.optimize(getRotationEncoderPosition());
        speedClosedLoopController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        rotationClosedLoopController.setReference(desiredState.angle.getDegrees(), ControlType.kPosition);
    }

    public void seed() {
        setRelativeEncoderPosition(getAbsoluteEncoderPosition());
    }

}
