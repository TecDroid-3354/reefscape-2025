package net.tecdroid.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import net.tecdroid.util.*;
import net.tecdroid.util.geometry.Wheel;

import static edu.wpi.first.units.Units.*;

public class IntakeController {
    private final TalonFX intakeMotorController;
    private final Config intakeConfig;
    public IntakeController(Config intakeConfig) {
        this.intakeConfig = intakeConfig;

        // Initialized both algae and coral motors
        intakeMotorController = new TalonFX(intakeConfig.Identifiers.intakeMotorId.getId());

        // Configures intake motor
        intakeMotorInterface();
    }

    // ////////////// //
    // INTAKE CONTROL //
    // ////////////// //

    public void enableIntake(LinearVelocity wheelSpeed) {
        /* Takes a wheel linear velocity and applies a motor angular velocity */
        // Checks that the desired speed is within the allowed range
        if (wheelSpeed.lte(intakeConfig.ControlConstants.intakeMaxWheelLinearVelocity)) {
            enableClosedIntake(intakeWheelLinearVelocityToMotorAngularVelocity(wheelSpeed));
        }
    }

    public void enableClosedIntake(AngularVelocity motorAngularVelocity) {
        /* Applies a voltage based on the desired motor angular velocity */
        intakeMotorController.setControl(
                new VelocityVoltage(motorAngularVelocity)
        );
    }

    public void retainAlgaeIntake() {
        /* Applies the minimum voltage to ensure algae does not go out the intake */
        intakeMotorController.setControl(
                new VoltageOut(intakeConfig.ControlConstants.retainAlgaeMinimumVoltage)
        );
    }

    public void enableOuttake(LinearVelocity wheelSpeed) {
        /* Takes a wheel linear velocity and applies the inverse of a motor angular velocity */
        // Checks that the desired speed is within the allowed range
        if (wheelSpeed.lte(intakeConfig.ControlConstants.intakeMaxWheelLinearVelocity)) {
            enableOuttake(intakeWheelLinearVelocityToMotorAngularVelocity(wheelSpeed));
        }
    }

    private void enableOuttake(AngularVelocity motorAngularVelocity) {
        /* Applies a voltage based on the inverse of the desired angular velocity */
        intakeMotorController.setControl(
                new VelocityVoltage(motorAngularVelocity.unaryMinus())
        );
    }

    public void stopIntake() { intakeMotorController.setVoltage(0.0); }

    // ///// //
    // Utils //
    // ///// //

    private AngularVelocity intakeWheelLinearVelocityToMotorAngularVelocity(LinearVelocity wheelSpeed) {
        /* Converts the wheel linear velocity to angular velocity and then multiplies by the gear ratio */
        return RotationsPerSecond.of(intakeConfig.PhysicalDescription.intakeMotorGearRatio.unapply(
                intakeConfig.PhysicalDescription.intakeWheel
                        .linearVelocityToAngularVelocity(wheelSpeed).in(RotationsPerSecond)
        ));
    }

    // ///////////// //
    // Configuration //
    // ///////////// //

    private void intakeMotorInterface() {
        /* Sets all algae motor's configurations */

        // Initialize configuration object
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();

        // Sets the motor to brake and decides whether to invert it or not
        intakeMotorConfig.MotorOutput.withNeutralMode(
                intakeConfig.ControlConstants.intakeMotorNeutralMode
        ).withInverted(
                intakeConfig.PhysicalDescription.intakeMotorGearRatio.transformRotation(
                        intakeConfig.Conventions.intakeMotorRotationalPositiveDirection
                ).toInvertedValue()
        );

        // Limits the motor's current
        intakeMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(intakeConfig.Limits.intakeMotorCurrentLimit);

        // Sets the time the motor should take to get to the desired speed / position
        intakeMotorConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(intakeConfig.ControlConstants.intakeRampRate);

        // Clear all sticky faults when initializing
        intakeMotorController.clearStickyFaults();

        // Apply the configuration
        intakeMotorController.getConfigurator().apply(intakeMotorConfig);
    }


    public record DeviceIdentifiers(NumericId intakeMotorId) {}
    public record DeviceProperties(MotorProperties intakeMotorProperties) {}
    public record DeviceLimits(Current intakeMotorCurrentLimit) {}
    public record DeviceConventions(RotationalDirection intakeMotorRotationalPositiveDirection) {}
    public record PhysicalDescription(GearRatio intakeMotorGearRatio, Wheel intakeWheel) {}
    public record ControlConstants(
            LinearVelocity intakeMaxWheelLinearVelocity, Voltage retainAlgaeMinimumVoltage,
            NeutralModeValue intakeMotorNeutralMode, Time intakeRampRate) {}
    public record Config(
            DeviceIdentifiers Identifiers, DeviceProperties Properties, DeviceLimits Limits,
            DeviceConventions Conventions, PhysicalDescription PhysicalDescription, ControlConstants ControlConstants
    ) {}
}
