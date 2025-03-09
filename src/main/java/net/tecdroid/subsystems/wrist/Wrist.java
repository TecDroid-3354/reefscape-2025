package net.tecdroid.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.tecdroid.subsystems.generic.VoltageControlledSubsystem;
import org.jetbrains.annotations.NotNull;

import static edu.wpi.first.units.Units.*;

public class Wrist extends SubsystemBase implements VoltageControlledSubsystem {
    private final TalonFX motorController;
    private final DutyCycleEncoder absoluteEncoder;
    private final WristConfig config;

    public Wrist(WristConfig config) {
        this.config = config;

        motorController = new TalonFX(this.config.getMotorControllerId().getId());
        absoluteEncoder = new DutyCycleEncoder(this.config.getAbsoluteEncoderPort().getId());

        // Configure motors
        configureMotorInterface();
    }

    @Override
    public void setVoltage(@NotNull Voltage voltage) {
        VoltageOut request = new VoltageOut(voltage);
        motorController.setControl(request);
    }

    @NotNull
    @Override
    public Command setVoltageCommand(@NotNull Voltage voltage) {
        return VoltageControlledSubsystem.super.setVoltageCommand(voltage);
    }

    @Override
    public void stop() {
        VoltageControlledSubsystem.super.stop();
    }

    @NotNull
    @Override
    public Command stopCommand() {
        return VoltageControlledSubsystem.super.stopCommand();
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
        motorController.setControl(request);
    }

    private Angle getMotorPosition() {
        /* Returns the leading motor's encoder reading as an Angle object from rotations */
        return motorController.getPosition().getValue();
    }

    public Angle getAngle() {
        /* Gets the leading motor's encoder reading and applies the gear ratio to obtain the wrist angle */
        return config.getGearRatio().apply(getMotorPosition());
    }

    private Angle getAbsoluteAngle() {
        Angle reportedAngle = Rotations.of(absoluteEncoder.get()).minus(config.getEncoderOffset());

        if (config.getAbsoluteEncoderIsInverted()) {
            reportedAngle = Rotations.of(1.0).minus(reportedAngle);
        }

        return config.getGearRatio().apply(reportedAngle);
    }

    public void matchMotorEncoderAngleToAbsoluteEncoderAngle() {
        motorController.setPosition(getAbsoluteAngle());
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
                .withMotionMagicCruiseVelocity(config.getGearRatio().unapply(config.getMotionTargets().getCruiseVelocity()))
                .withMotionMagicAcceleration(config.getGearRatio().unapply(config.getMotionTargets().getAcceleration()))
                .withMotionMagicJerk(config.getGearRatio().unapply(config.getMotionTargets().getJerk()));

        // Clear all sticky faults when initializing
        motorController.clearStickyFaults();

        // Apply the configuration
        motorController.getConfigurator().apply(wristMotorConfig);
    }

}
