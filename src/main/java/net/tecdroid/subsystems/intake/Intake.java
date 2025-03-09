package net.tecdroid.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.tecdroid.subsystems.generic.VoltageControlledSubsystem;
import org.jetbrains.annotations.NotNull;

public class Intake extends SubsystemBase implements VoltageControlledSubsystem {
    private final TalonFX motorController;
    private final IntakeConfig config;

    public Intake(IntakeConfig config) {
        this.config = config;

        // Initialized both algae and coral motors
        motorController = new TalonFX(config.getMotorControllerId().getId());

        // Configures intake motor
        configureMotorInterface();
    }

    // ////////////// //
    // INTAKE CONTROL //
    // ////////////// //

    @Override
    public void setVoltage(@NotNull Voltage voltage) {
        VoltageOut request = new  VoltageOut(voltage);
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

    // ///////////// //
    // Configuration //
    // ///////////// //

    private void configureMotorInterface() {
        /* Sets all algae motor's configurations */

        // Initialize configuration object
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();

        // Sets the motor to brake and decides whether to invert it or not
        intakeMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        // Limits the motor's current
        intakeMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.getCurrentLimit());

        // Clear all sticky faults when initializing
        motorController.clearStickyFaults();

        // Apply the configuration
        motorController.getConfigurator().apply(intakeMotorConfig);
    }

}
