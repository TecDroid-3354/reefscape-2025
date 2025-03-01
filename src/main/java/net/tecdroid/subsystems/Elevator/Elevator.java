package net.tecdroid.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.tecdroid.util.GearRatio;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {
    TalonFX mLeftMotor;
    TalonFX mRightMotor;
    DutyCycleEncoder absoluteEncoder;

    public Elevator() {
        mLeftMotor = new TalonFX(ElevatorConstants.Elevator.kLeftMotorID);
        mRightMotor = new TalonFX(ElevatorConstants.Elevator.kRightMotorID);

        // Motion profile config
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ElevatorConstants.ElevatorCoefficients.kS;
        slot0Configs.kV = ElevatorConstants.ElevatorCoefficients.kV;
        slot0Configs.kA = ElevatorConstants.ElevatorCoefficients.kA;
        slot0Configs.kP = ElevatorConstants.ElevatorCoefficients.kP;
        slot0Configs.kI = ElevatorConstants.ElevatorCoefficients.kI;
        slot0Configs.kD = ElevatorConstants.ElevatorCoefficients.kD;
        // feedforward: https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/closed-loop-requests.html

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // Config based on: https://www.youtube.com/watch?v=Ew3dxj9uIdY

        // Configure limits
        var limitsConfig = talonFXConfigs.CurrentLimits;

        limitsConfig.StatorCurrentLimit = 40;
        limitsConfig.StatorCurrentLimitEnable = true;

        // Apply config
        mLeftMotor.getConfigurator().apply(talonFXConfigs);
        mRightMotor.getConfigurator().apply(talonFXConfigs);

        // Set brake
        mLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        mRightMotor.setNeutralMode(NeutralModeValue.Brake);

        // Invert leader motor (right)
        //mRightMotor.setInverted(true);

        // Make the left motor to follow the right one
        mLeftMotor.setControl(new Follower(mRightMotor.getDeviceID(), false));

        // Absolute Encoder
        absoluteEncoder = new DutyCycleEncoder(ElevatorConstants.AbsoluteEncoder.kAbsoluteEncoderChannel);
        absoluteEncoder.setInverted(ElevatorConstants.AbsoluteEncoder.kIsInverted);

        // set the position of the motor as the absolute encoder in the innit
        //resetMotorPositionsToAbsoluteEncoderPosition();
    }

    public Angle getAbsoluteEncoderRot() {
        return Rotations.of(absoluteEncoder.get() - ElevatorConstants.AbsoluteEncoder.kOffsetPosition);
    }

    public Distance getAbsoluteEncoderDistance() {
        return Distance.ofBaseUnits(getAbsoluteEncoderRot().in(Rotations) * ElevatorConstants.ElevatorReductions.elevatorInchesPerRev, Inches);
    }

    public Angle getRightMotorRot() {
        return Rotations.of(mRightMotor.getPosition().getValueAsDouble() * ElevatorConstants.ElevatorReductions.elevatorGearRatioConversionFactor);
    }

    public Distance getRightMotorDistance() {
        return Distance.ofBaseUnits(getRightMotorRot().in(Rotations) * ElevatorConstants.ElevatorReductions.elevatorInchesPerRev, Inches);
    }

    public void resetMotorsPositionsToAbsoluteEncoderPosition() {
        mRightMotor.setPosition(getAbsoluteEncoderRot().in(Rotations) * ElevatorConstants.ElevatorReductions.elevatorGearRatio);
        mLeftMotor.setPosition(getAbsoluteEncoderRot().in(Rotations) * ElevatorConstants.ElevatorReductions.elevatorGearRatio);
    }

    public Command goToPosition(Distance requestedPosition) {
        return runOnce(
                () -> {
                    // pre-process position (inches to rotations)
                    Angle requestedRotations = Rotations.of(requestedPosition.in(Inches) * ElevatorConstants.ElevatorReductions.elevatorGearRatio
                            / ElevatorConstants.ElevatorReductions.elevatorInchesPerRev);

                    // create a Motion Magic request, voltage output
                    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

                    // set target position to 100 rotations
                    mRightMotor.setControl(m_request.withPosition(requestedRotations.in(Rotations)));
                });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
