package net.tecdroid.subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {
    TalonFX mLeftMotor;
    TalonFX mRightMotor;
    DutyCycleEncoder absoluteEncoder;

    public Angle getAbsoluteEncoderRot() {
        return Rotations.of(absoluteEncoder.get() - ElevatorConstants.AbsoluteEncoder.OFFSET_POSITION);
    }

    public Distance getAbsoluteEncoderDistance() {
        return Distance.ofBaseUnits(getAbsoluteEncoderRot().in(Rotations) * ElevatorConstants.ElevatorReductions.ELEVATOR_INCHES_PER_REV, Inches);
    }

    public Angle getRightMotorRot() {
        return Rotations.of(mRightMotor.getPosition().getValueAsDouble() * ElevatorConstants.ElevatorReductions.ELEVATOR_GEAR_RATIO_CONVERSION_FACTOR);
    }

    public Distance getRightMotorDistance() {
        return Distance.ofBaseUnits(getRightMotorRot().in(Rotations) * ElevatorConstants.ElevatorReductions.ELEVATOR_INCHES_PER_REV, Inches);
    }

    public void resetMotorsPositionsToAbsoluteEncoderPosition() {
        mRightMotor.setPosition(getAbsoluteEncoderRot().in(Rotations) * ElevatorConstants.ElevatorReductions.ELEVATOR_GEAR_RATIO);
        mLeftMotor.setPosition(getAbsoluteEncoderRot().in(Rotations) * ElevatorConstants.ElevatorReductions.ELEVATOR_GEAR_RATIO);
    }

    public void ElevatorConfig() {
        mLeftMotor = new TalonFX(ElevatorConstants.ElevatorMotors.LEFT_MOTOR_ID);
        mRightMotor = new TalonFX(ElevatorConstants.ElevatorMotors.RIGHT_MOTOR_ID);

        // Motion profile config
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ElevatorConstants.ElevatorCoefficients.S;
        slot0Configs.kV = ElevatorConstants.ElevatorCoefficients.V;
        slot0Configs.kA = ElevatorConstants.ElevatorCoefficients.A;
        slot0Configs.kP = ElevatorConstants.ElevatorCoefficients.P;
        slot0Configs.kI = ElevatorConstants.ElevatorCoefficients.I;
        slot0Configs.kD = ElevatorConstants.ElevatorCoefficients.D;
        // feedforward: https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/closed-loop-requests.html

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.ElevatorMotionMagicSettings.MOTION_MAGIC_CRUISE_VELOCITY; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.ElevatorMotionMagicSettings.MOTION_MAGIC_ACCELERATION; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = ElevatorConstants.ElevatorMotionMagicSettings.MOTION_MAGIC_JERK; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // Config based on: https://www.youtube.com/watch?v=Ew3dxj9uIdY

        // Configure limits
        var limitsConfig = talonFXConfigs.CurrentLimits;

        limitsConfig.StatorCurrentLimit = ElevatorConstants.ElevatorMotors.AMP_LIMITS;
        limitsConfig.StatorCurrentLimitEnable = true;

        // Apply config
        mLeftMotor.getConfigurator().apply(talonFXConfigs);
        mRightMotor.getConfigurator().apply(talonFXConfigs);

        // Set brake
        mLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        mRightMotor.setNeutralMode(NeutralModeValue.Brake);

        // Make the left motor to follow the right one
        mLeftMotor.setControl(new Follower(mRightMotor.getDeviceID(),
                ElevatorConstants.ElevatorMotors.IS_INVERTED));

        // Absolute Encoder
        absoluteEncoder = new DutyCycleEncoder(ElevatorConstants.AbsoluteEncoder.ABSOLUTE_ENCODER_CHANNEL);
        absoluteEncoder.setInverted(ElevatorConstants.AbsoluteEncoder.IS_INVERTED);
    }

    public Elevator() {
        ElevatorConfig();

        // set the position of the motor as the absolute encoder in the innit
        //resetMotorPositionsToAbsoluteEncoderPosition();
    }

    public Command goToPosition(Distance requestedPosition) {
        return runOnce(
                () -> {
                    // pre-process position (inches to rotations)
                    Angle requestedRotations = Rotations.of(requestedPosition.in(Inches) * ElevatorConstants.ElevatorReductions.ELEVATOR_GEAR_RATIO
                            / ElevatorConstants.ElevatorReductions.ELEVATOR_INCHES_PER_REV);

                    // create a Motion Magic request, voltage output
                    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

                    // set target position to 100 rotations
                    mRightMotor.setControl(m_request.withPosition(requestedRotations.in(Rotations)));
                });
    }

    // Test functions
    public void moveMotors(double voltage) {
        mLeftMotor.setVoltage(voltage);
        mRightMotor.setVoltage(voltage);
    }

    public void stopMotors() {
        mLeftMotor.setVoltage(0.0);
        mRightMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
