package net.tecdroid.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.tecdroid.util.*;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {
    TalonFX mLeftMotor;
    TalonFX mRightMotor;
    ElevatorConfig elevatorConfig = ElevatorConfiguration.elevatorConfig;


    public Angle getRightMotorRot() {
        return Rotations.of(elevatorConfig.gearRatio.motorGearRatio.apply(mRightMotor.getPosition().getValueAsDouble()));
    }

    public Distance getRightMotorDistance() {
        return Distance.ofBaseUnits(getRightMotorRot().in(Rotations) * elevatorConfig.gearRatio.elevatorInchesPerRev.in(Inches), Inches);
    }

    public void ElevatorConfig() {
        mLeftMotor = new TalonFX(elevatorConfig.deviceIdentifier.leftMotorId.getId());
        mRightMotor = new TalonFX(elevatorConfig.deviceIdentifier.rightMotorId.getId());

        // Motion profile config
        var talonFXConfigs = new TalonFXConfiguration();

        // Invert Motors
        talonFXConfigs.MotorOutput.withInverted(elevatorConfig.motorProperties.leaderMotorInvertedType);

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = elevatorConfig.coefficients.svagGains.getS();
        slot0Configs.kV = elevatorConfig.coefficients.svagGains.getV();
        slot0Configs.kA = elevatorConfig.coefficients.svagGains.getA();
        slot0Configs.kP = elevatorConfig.coefficients.pidfCoefficients.getP();
        slot0Configs.kI = elevatorConfig.coefficients.pidfCoefficients.getI();
        slot0Configs.kD = elevatorConfig.coefficients.pidfCoefficients.getD();
        // feedforward: https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/closed-loop-requests.html

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = elevatorConfig.motionMagicProperties.motionMagicSettings.getMotionMagicCruiseVelocity(); // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = elevatorConfig.motionMagicProperties.motionMagicSettings.getMotionMagicAcceleration(); // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = elevatorConfig.motionMagicProperties.motionMagicSettings.getMotionMagicJerk(); // Target jerk of 1600 rps/s/s (0.1 seconds)

        // Config based on: https://www.youtube.com/watch?v=Ew3dxj9uIdY

        // Configure limits
        var limitsConfig = talonFXConfigs.CurrentLimits;

        limitsConfig.StatorCurrentLimit = elevatorConfig.motorProperties.ampLimits.in(Amps);
        limitsConfig.StatorCurrentLimitEnable = true;

        // Apply config
        mLeftMotor.getConfigurator().apply(talonFXConfigs);
        mRightMotor.getConfigurator().apply(talonFXConfigs);

        // Set brake
        mLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        mRightMotor.setNeutralMode(NeutralModeValue.Brake);

        // Make the left motor to follow the right one
        mLeftMotor.setControl(new Follower(mRightMotor.getDeviceID(),
                elevatorConfig.motorProperties.followerMotorInverted));
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
                    Angle requestedRotations = Rotations.of(elevatorConfig.gearRatio.motorGearRatio.unapply(
                            requestedPosition.in(Inches) / elevatorConfig.gearRatio.elevatorInchesPerRev.in(Inches)
                    ));

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

    public record DeviceIdentifier(NumericId leftMotorId, NumericId rightMotorId, DigitId limitSwitchChannel) {}
    public record MotorProperties(Current ampLimits, InvertedValue leaderMotorInvertedType, boolean followerMotorInverted) {}
    public record ElevatorGearRatio(GearRatio motorGearRatio, Distance elevatorInchesPerRev) {}
    public record EncoderProperties(Boolean absoluteEncoderReversed, Angle encoderOffsets) {}
    public record Coefficients(PidfCoefficients pidfCoefficients, SvagGains svagGains) {}
    public record MotionMagicProperties(MotionMagicSettings motionMagicSettings) {}

    public record ElevatorConfig(DeviceIdentifier deviceIdentifier, MotorProperties motorProperties,
                                 ElevatorGearRatio gearRatio, EncoderProperties encoderProperties,
                                 Coefficients coefficients,
                                 MotionMagicProperties motionMagicProperties) {}


}
