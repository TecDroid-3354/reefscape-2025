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
    private final TalonFX algaeIntakeMotorController;
    private final TalonFX coralIntakeMotorController;
    private final Config intakeConfig;
    public IntakeController(Config intakeConfig) {
        this.intakeConfig = intakeConfig;

        // Initialized both algae and coral motors
        algaeIntakeMotorController = new TalonFX(intakeConfig.Identifiers.algaeIntakeMotorID.getId());
        coralIntakeMotorController = new TalonFX(intakeConfig.Identifiers.coralIntakeMotorID.getId());

        // Configures algae and coral motors
        algaeMotorInterface();
        coralMotorInterface();
    }

    // //////////// //
    // ALGAE INTAKE //
    // //////////// //

    public void enableAlgaeIntake(LinearVelocity wheelSpeed) {
        /* Takes a wheel linear velocity and applies a motor angular velocity */
        // Checks that the desired speed is within the allowed range
        if (wheelSpeed.lte(intakeConfig.ControlConstants.algaeIntakeMaxWheelLinearVelocity)) {
            enableAlgaeIntake(algaeWheelLinearVelocityToMotorAngularVelocity(wheelSpeed));
        }
    }

    private void enableAlgaeIntake(AngularVelocity motorAngularVelocity) {
        /* Applies a voltage based on the desired motor angular velocity */
        algaeIntakeMotorController.setControl(
                new VelocityVoltage(motorAngularVelocity)
        );
    }

    public void retainAlgaeIntake() {
        /* Applies the minimum voltage to ensure algae does not go out the intake */
        algaeIntakeMotorController.setControl(
                new VoltageOut(intakeConfig.ControlConstants.retainAlgaeMinimumVoltage)
        );
    }

    public void enableAlgaeOuttake(LinearVelocity wheelSpeed) {
        /* Takes a wheel linear velocity and applies the inverse of a motor angular velocity */
        // Checks that the desired speed is within the allowed range
        if (wheelSpeed.lte(intakeConfig.ControlConstants.algaeIntakeMaxWheelLinearVelocity)) {
            enableAlgaeOuttake(algaeWheelLinearVelocityToMotorAngularVelocity(wheelSpeed));
        }
    }

    private void enableAlgaeOuttake(AngularVelocity motorAngularVelocity) {
        /* Applies a voltage based on the inverse of the desired angular velocity */
        algaeIntakeMotorController.setControl(
                new VelocityVoltage(motorAngularVelocity.unaryMinus())
        );
    }

    public void stopAlgaeIntake() { algaeIntakeMotorController.setVoltage(0.0); }

    // //////////// //
    // CORAL INTAKE //
    // //////////// //

    public void enableCoralIntake(LinearVelocity wheelSpeed) {
        /* Takes a wheel linear velocity and applies a motor angular velocity */
        enableCoralIntake(coralWheelLinearVelocityToMotorAngularVelocity(wheelSpeed));
    }

    private void enableCoralIntake(AngularVelocity motorAngularVelocity) {
        /* Applies a voltage based on the desired motor angular velocity */
        coralIntakeMotorController.setControl(
                new VelocityVoltage(motorAngularVelocity)
        );
    }
    public void enableCoralOuttake(LinearVelocity wheelSpeed) {
        /* Calls enableCoralIntake(), since outtake and intake share same rotational direction */
        enableCoralIntake(wheelSpeed);
    }

    public void stopCoralIntake() { coralIntakeMotorController.setVoltage(0.0); }

    // ///// //
    // Utils //
    // ///// //

    private AngularVelocity algaeWheelLinearVelocityToMotorAngularVelocity(LinearVelocity wheelSpeed) {
        /* Converts the wheel linear velocity to angular velocity and then multiplies by the gear ratio */
        return RotationsPerSecond.of(intakeConfig.PhysicalDescription.algaeIntakeMotorGearRatio.unapply(
                intakeConfig.PhysicalDescription.algaeIntakeWheel
                        .linearVelocityToAngularVelocity(wheelSpeed).in(RotationsPerSecond)
        ));
    }

    private AngularVelocity coralWheelLinearVelocityToMotorAngularVelocity(LinearVelocity wheelSpeed) {
        /* Converts the wheel linear velocity to angular velocity and then multiplies by the gear ratio */
        return RotationsPerSecond.of(intakeConfig.PhysicalDescription.coralIntakeMotorGearRatio.unapply(
                intakeConfig.PhysicalDescription.coralIntakeWheel
                        .linearVelocityToAngularVelocity(wheelSpeed).in(RotationsPerSecond)
        ));
    }

    // ///////////// //
    // Configuration //
    // ///////////// //

    public void algaeMotorInterface() {
        /* Sets all algae motor's configurations */

        // Initialize configuration object
        TalonFXConfiguration algaeMotorConfig = new TalonFXConfiguration();

        // Sets the motor to brake and decides whether to invert it or not
        algaeMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake).withInverted(
                intakeConfig.PhysicalDescription.algaeIntakeMotorGearRatio.transformRotation(
                        intakeConfig.Conventions.algaeIntakeMotorRotationalPositiveDirection
                ).toInvertedValue()
        );

        // Limits the motor's current
        algaeMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(intakeConfig.Limits.algaeIntakeMotorCurrentLimit);

        // Sets the time the motor should take to get to the desired speed / position
        algaeMotorConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(intakeConfig.ControlConstants.intakeRampRate);

        // Clear all sticky faults when initializing
        algaeIntakeMotorController.clearStickyFaults();

        // Apply the configuration
        algaeIntakeMotorController.getConfigurator().apply(algaeMotorConfig);
    }
    public void coralMotorInterface() {
        /* Sets all coral motor's configurations */

        // Initialize configuration object
        TalonFXConfiguration coralMotorConfig = new TalonFXConfiguration();

        // Sets the motor to brake and decides whether to invert it or not
        coralMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake).withInverted(
                intakeConfig.PhysicalDescription.coralIntakeMotorGearRatio.transformRotation(
                        intakeConfig.Conventions.coralIntakeMotorRotationalPositiveDirection
                ).toInvertedValue()
        );

        // Limits the motor's current
        coralMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(intakeConfig.Limits.coralIntakeMotorCurrentLimit);

        // Sets the time the motor should take to get to the desired speed
        coralMotorConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(intakeConfig.ControlConstants.intakeRampRate);

        // Clear all sticky faults when initializing
        coralIntakeMotorController.clearStickyFaults();

        // Apply the configuration
        coralIntakeMotorController.getConfigurator().apply(coralMotorConfig);
    }


    public record DeviceIdentifiers(NumericId algaeIntakeMotorID, NumericId coralIntakeMotorID) {}
    public record DeviceProperties(MotorProperties algaeIntakeMotorProperties, MotorProperties coralIntakeMotorProperties) {}
    public record DeviceLimits(Current algaeIntakeMotorCurrentLimit, Current coralIntakeMotorCurrentLimit) {}
    public record DeviceConventions(RotationalDirection algaeIntakeMotorRotationalPositiveDirection,
                                    RotationalDirection coralIntakeMotorRotationalPositiveDirection) {}
    public record PhysicalDescription(
            GearRatio algaeIntakeMotorGearRatio, GearRatio coralIntakeMotorGearRatio,
            Wheel algaeIntakeWheel, Wheel coralIntakeWheel) {}
    public record ControlConstants(
            LinearVelocity algaeIntakeMaxWheelLinearVelocity, LinearVelocity coralIntakeMaxWheelLinearVelocity,
            Voltage retainAlgaeMinimumVoltage, Time intakeRampRate) {}
    public record Config(
            DeviceIdentifiers Identifiers, DeviceProperties Properties, DeviceLimits Limits,
            DeviceConventions Conventions, PhysicalDescription PhysicalDescription, ControlConstants ControlConstants
    ) {}
}
