package net.tecdroid.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static edu.wpi.first.units.Units.*;

public class Wrist {
    private final TalonFX wristMotorController;
    private final DutyCycleEncoder wristEncoder;
    private final WristConfig config;

    public Wrist(WristConfig config) {
        this.config = config;

        wristMotorController = new TalonFX(this.config.getMotorControllerId().getId());
        // wrist encoder --> throughbore (roboRIO connected)
        wristEncoder = new DutyCycleEncoder(this.config.getAbsoluteEncoderPort().getId());

        // Configure motors
        configureMotorInterface();
        matchMotorEncoderAngleToAbsoluteEncoderAngle();
    }

    public void setWristAngle(Angle angle) {
        Angle targetAngle = angle;

        if (targetAngle.lt(config.getMinimumAngle())) {
            targetAngle = config.getMinimumAngle();
        }

        if (targetAngle.gt(config.getMaximumAngle())) {
            targetAngle = config.getMaximumAngle();
        }

        MotionMagicVoltage request = new MotionMagicVoltage(config.getGearRatio().unapply(targetAngle)).withSlot(0);
        wristMotorController.setControl(request);
    }

    private Angle getMotorPosition() {
        /* Returns the leading motor's encoder reading as an Angle object from rotations */
        return wristMotorController.getPosition().getValue();
    }

    public Angle getAngle() {
        /* Gets the leading motor's encoder reading and applies the gear ratio to obtain the wrist angle */
        return Rotations.of(config.getGearRatio().apply(getMotorPosition().in(Rotations)));
    }

    private Angle getAbsoluteAngle() {
        Angle reportedAngle = Rotations.of(wristEncoder.get()).minus(config.getEncoderOffset());

        if (config.getAbsoluteEncoderIsInverted()) {
            return Rotations.of(1.0).minus(reportedAngle);
        }

        return reportedAngle;
    }

    private void matchMotorEncoderAngleToAbsoluteEncoderAngle() {
        wristMotorController.setPosition(getAbsoluteAngle());
    }

    private void publishToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Wrist");
        tab.addDouble("Wrist Angle", () -> getAngle().in(Degrees));
        tab.addDouble("Wrist Absolute Angle", () -> getAbsoluteAngle().in(Degrees));
    }

    // ///////////// //
    // Configuration //
    // ///////////// //

    private void configureMotorInterface() {
        /* Sets all motor's configurations */

        // Initialize configuration object
        TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();

        // Sets the motor to brake and decides whether to invert it or not
        wristMotorConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.getGearRatio().transformRotation(config.getPositiveDirection()).toInvertedValue());

        // Limits the motor's current
        wristMotorConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.getMotorCurrentLimit());

        // Sets all PID, SVAG control constants
        wristMotorConfig.Slot0
                .withKP(config.getControlGains().getP())
                .withKI(config.getControlGains().getI())
                .withKD(config.getControlGains().getD())
                .withKS(config.getControlGains().getS())
                .withKV(config.getControlGains().getV())
                .withKA(config.getControlGains().getA())
                .withKG(config.getControlGains().getG());

        wristMotorConfig.MotionMagic
                .withMotionMagicCruiseVelocity(config.getGearRatio().unapply(config.getMotionMagicCoefficients().getCruiseVelocity()))
                .withMotionMagicAcceleration(config.getGearRatio().unapply(config.getMotionMagicCoefficients().getAcceleration()))
                .withMotionMagicJerk(config.getGearRatio().unapply(config.getMotionMagicCoefficients().getJerk()));

        // Clear all sticky faults when initializing
        wristMotorController.clearStickyFaults();

        // Apply the configuration
        wristMotorController.getConfigurator().apply(wristMotorConfig);
    }
}
