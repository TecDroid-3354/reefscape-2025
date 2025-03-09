
package net.tecdroid.subsystems.elevatorjoint;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.util.units.NumericKt.clamp;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.tecdroid.subsystems.generic.VoltageControlledSubsystem;
import net.tecdroid.subsystems.generic.WithAbsoluteEncoders;
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder;
import org.jetbrains.annotations.NotNull;

public class ElevatorJoint extends SubsystemBase implements VoltageControlledSubsystem, WithAbsoluteEncoders {
    private final TalonFX leadMotorController; // right motor
    private final TalonFX followerMotorController; // left motor
    private final ThroughBoreAbsoluteEncoder absoluteEncoder;
    private final ElevatorJointConfig config;

    public ElevatorJoint(ElevatorJointConfig config) {
        this.config = config;

        leadMotorController = new TalonFX(config.getLeadMotorControllerId().getId());
        followerMotorController = new TalonFX(config.getFollowerMotorId().getId());
        absoluteEncoder = new ThroughBoreAbsoluteEncoder(config.getAbsoluteEncoderPort(), config.getAbsoluteEncoderIsInverted());

        configureMotorsInterface();
    }

    @Override
    public void setVoltage(@NotNull Voltage voltage) {
        VoltageOut request = new VoltageOut(voltage);
        leadMotorController.setControl(request);
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

    public void setTargetAngle(Angle angle) {
        Angle targetAngle = (Angle) clamp(config.getMinimumAngle(), config.getMaximumAngle(), angle);
        MotionMagicVoltage request = new MotionMagicVoltage(config.getGearRatio().unapply(targetAngle));
        leadMotorController.setControl(request);
    }

    public Command setTargetAngleCommand(Angle angle) {
        return Commands.runOnce(() -> {
            setTargetAngle(angle);
        });
    }

    public Angle getAngle() {
        return config.getGearRatio().apply(leadMotorController.getPosition().getValue());
    }

    public Angle getAbsoluteAngle() {
        return config.getGearRatio().apply(absoluteEncoder.getPosition());
    }

    @Override
    public void matchRelativeEncodersToAbsoluteEncoders() {
        leadMotorController.setPosition(config.getGearRatio().unapply(getAbsoluteAngle()));
    }

    public void configureMotorsInterface() {
        TalonFXConfiguration motorsConfig = new TalonFXConfiguration();

        motorsConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.getGearRatio().transformRotation(config.getPositiveDirection()).toInvertedValue());

        motorsConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.getCurrentLimit());

        motorsConfig.Slot0
                .withKP(config.getControlGains().getP())
                .withKI(config.getControlGains().getI())
                .withKD(config.getControlGains().getD())
                .withKS(config.getControlGains().getS())
                .withKV(config.getControlGains().getV())
                .withKA(config.getControlGains().getA())
                .withKG(config.getControlGains().getG());

        motorsConfig.MotionMagic
                .withMotionMagicCruiseVelocity(config.getMotionTargets().getCruiseVelocity())
                .withMotionMagicAcceleration(config.getMotionTargets().getAcceleration())
                .withMotionMagicJerk(config.getMotionTargets().getJerk());

        leadMotorController.clearStickyFaults();
        followerMotorController.clearStickyFaults();

        leadMotorController.getConfigurator().apply(motorsConfig);
        followerMotorController.getConfigurator().apply(motorsConfig);

        followerMotorController.setControl(new Follower(leadMotorController.getDeviceID(), true));
    }

}


