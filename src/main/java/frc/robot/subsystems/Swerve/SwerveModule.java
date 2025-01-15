// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

    public SwerveModule(int speedMotorID, int rotationMotorID, int absoluteEncoderID) {
        // Setting the controller, encoder and closed loop controller for the speed motor
        speedMotorController = new SparkMax(speedMotorID, MotorType.kBrushless);
        speedMotorEncoder = speedMotorController.getEncoder();
        speedClosedLoopController = speedMotorController.getClosedLoopController();

        // Spark configuration (Idle mode, PIDF, Continous Input)
        SparkMaxConfig speedMotorConfig = new SparkMaxConfig();

        speedMotorConfig
            .idleMode(IdleMode.kBrake)
            .encoder.positionConversionFactor(UnitsConversion.SPEED_ENCODER_ROTATIONS_TO_METERS)
            .velocityConversionFactor(UnitsConversion.SPEED_ENCODER_RPM_TO_METERS_PER_SECOND);

        speedMotorConfig.closedLoop.pidf(
                SparkPID.Speed.kP, 
                SparkPID.Speed.kI, 
                SparkPID.Speed.kD, 
                SparkPID.Speed.kFF
            );
        
        // Apply the configuration
        speedMotorController.configure(speedMotorConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // ! Setting rotation spark basic configuration, including PID and idleMode
        rotationMotorController = new SparkMax(rotationMotorID, MotorType.kBrushless);
        rotationMotorEncoder = rotationMotorController.getEncoder();
        rotationClosedLoopController = rotationMotorController.getClosedLoopController();

        SparkMaxConfig rotationMotorConfig = new SparkMaxConfig();
        
        //TODO: Ver si utilizamos getVelocity(), caso contrario eliminar velocityConversionFactor()
        rotationMotorConfig
            .idleMode(IdleMode.kBrake)
            .encoder.positionConversionFactor(UnitsConversion.ROTATION_ENCODER_ROTATIONS_TO_RADIANS)
            .velocityConversionFactor(UnitsConversion.ROTATION_ENCODER_RPM_TO_RADIANS_PER_SECOND);

        rotationMotorConfig.closedLoop.pid(
            SparkPID.Rotation.kP,
            SparkPID.Rotation.kI,
            SparkPID.Rotation.kD
        ).positionWrappingEnabled(true).positionWrappingInputRange(-Math.PI, Math.PI); // Position wrapping —— Range resets when it surpasses either parameter 
        
        // Setting the configuration up for rotation spark
        rotationMotorController.configure(rotationMotorConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        absoluteEncoder = new CANcoder(absoluteEncoderID);
    }

    public double getSpeedEncoderVelocity() {
        /*Returns the encoder's reading in meters per second*/
        return speedMotorEncoder.getVelocity();
    }

    public double getSpeedEncoderPosition() {
        /*Returns the encoder's reading in  meters*/
        return speedMotorEncoder.getPosition();
    }
    
    public double getRotationEncoderPosition() {
        /*Returns the encoder's reading in Radians*/
        return rotationMotorEncoder.getPosition();
    }
  

}
