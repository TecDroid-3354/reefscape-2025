package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants.ModuleConstants;

import static java.lang.Math.PI;

public class SwerveModule {

    private final SparkMax driveController;
    private final SparkMax turningController;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final SparkClosedLoopController turningClosedLoopController;
    private final SparkClosedLoopController driveClosedLoopController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed) {

        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);
       
        driveController = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningController = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveClosedLoopController = driveController.getClosedLoopController();
        turningClosedLoopController = turningController.getClosedLoopController();

        // Drive Motor Spark config
        SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
        
        driveMotorConfig.inverted(driveMotorReversed).idleMode(IdleMode.kBrake);
        driveMotorConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
            .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        driveMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0).velocityFF(0.1);

        driveController.configure(driveMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Turning Motor Spark config
        SparkMaxConfig turningMotorConfig = new SparkMaxConfig();
        
        turningMotorConfig.inverted(turningMotorReversed).idleMode(IdleMode.kBrake);
        turningMotorConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
            .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        turningMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0).positionWrappingEnabled(true).positionWrappingInputRange(-PI, PI);
            
        turningController.configure(turningMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveController.getEncoder();
        turningEncoder = turningController.getEncoder();

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRotations() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getAbsoluteEncoderRad() {
        double angle = getAbsoluteEncoderRotations() * 2.0 * Math.PI;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        state.optimize(getState().angle);

        driveClosedLoopController.setReference(state.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
        turningClosedLoopController.setReference(state.angle.getRadians(), SparkBase.ControlType.kPosition);
    }

    public void stop() {
        driveController.set(0);
        turningController.set(0);
    }
}
