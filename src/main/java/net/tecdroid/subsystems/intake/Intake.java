package net.tecdroid.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import net.tecdroid.util.*;
import net.tecdroid.util.geometry.Wheel;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    private final TalonFX motorController;
    private final IntakeConfig config;
    private final DigitalInput beamBreak;
    private final Trigger beamBreakTrigger;

    public Intake(IntakeConfig config) {
        this.config = config;
        this.beamBreak = new DigitalInput(config.getBeamBreakId().getId());
        this.beamBreakTrigger = new Trigger(() -> !beamBreak.get());

        // Initialized both algae and coral motors
        motorController = new TalonFX(config.getMotorControllerId().getId());

        // Configures intake motor
        intakeMotorInterface();

        beamBreakTrigger.onTrue(setVoltageCommand(Volts.of(0.0)).andThen(Commands.print("AAA")));
    }

    // ////////////// //
    // INTAKE CONTROL //
    // ////////////// //

    public void setVoltage(Voltage voltage) {
        VoltageOut request = new  VoltageOut(voltage);
        motorController.setControl(request);
    }

    public Command setVoltageCommand(Voltage voltage) {
        return Commands.runOnce(() -> {
            setVoltage(voltage);
        }, this);
    }

    public void retainAlgae() {
        /* Applies the minimum voltage to ensure algae does not go out the intake */
        setVoltage(config.getVoltageToRetainAlgae());
    }

    private void enableOuttake(AngularVelocity motorAngularVelocity) {
        /* Applies a voltage based on the inverse of the desired angular velocity */
        motorController.setControl(
                new VelocityVoltage(motorAngularVelocity.unaryMinus())
        );
    }

    public void stopIntake() { motorController.setVoltage(0.0); }

    // ///////////// //
    // Configuration //
    // ///////////// //

    private void intakeMotorInterface() {
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
