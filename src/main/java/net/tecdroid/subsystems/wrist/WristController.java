package net.tecdroid.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import net.tecdroid.util.*;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class WristController {
    private final TalonFX wristMotorController;
    private final DutyCycleEncoder wristEncoder;
    private final Config wristConfig;
    private final Supplier<Boolean> isElevatorAngleDown;

    public WristController(Config wristConfig, Supplier<Boolean> isElevatorAngleDown) {
        this.wristConfig = wristConfig;
        this.isElevatorAngleDown = isElevatorAngleDown;

        // wrist encoder --> throughbore (roboRIO connected)
        wristEncoder = new DutyCycleEncoder(wristConfig.Identifiers.encoderID.getId());

        wristMotorController = new TalonFX(wristConfig.Identifiers.wristMotorID.getId());

        // Configure motors
        motorsInterface();
    }

    public Angle getWristAngle() {
        /* Gets the leading motor's encoder reading and applies the gear ratio to obtain the wrist angle */
        return Rotations.of(wristConfig.PhysicalDescription.motorsGearRatio.apply(getMotorEncoderAngle().in(Rotations)));
    }

    private Angle getMotorEncoderAngle() {
        /* Returns the leading motor's encoder reading as an Angle object from rotations */
        return wristMotorController.getPosition().getValue();
    }

    public void setWristAngle(Angle angle) {
        /* Takes the desired wrist angle, converts it to motor rotations and applies it */
        // Checks that the desired angle is within allowed range and that elevator angle is not down.
        // Always allows to move the wrist to {insert value} degrees or more.
        if (isAngleWithinRange(angle) &&
                (!isElevatorAngleDown.get() || angle.gte(Degrees.of(0.0)))) {
            wristMotorController.setControl(new PositionVoltage(
                    wristConfig.PhysicalDescription.motorsGearRatio.unapply(angle)
            ));
        }
    }

    private boolean isAngleWithinRange(Angle angle) {
        /* Makes sure that the desired wrist angle is within the allowed range */
        // gte() == greater than or equal to       lte() == less than or equal to
        return angle.gte(wristConfig.Limits.minimumAngleLimit) && angle.lte(wristConfig.Limits.maximumAngleLimit);
    }

    // TODO: GET THROUGHBORE OFFSET

    // ///////////// //
    // Configuration //
    // ///////////// //

    private void motorsInterface() {
        /* Sets all motor's configurations */

        // Initialize configuration object
        TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();

        // Sets the motor to brake and decides whether to invert it or not
        wristMotorConfig.MotorOutput.withNeutralMode(
                wristConfig.ControlConstants.motorNeutralMode
        ).withInverted(
                wristConfig.PhysicalDescription.motorsGearRatio.transformRotation(
                        wristConfig.Conventions.motorRotationalPositiveDirection
                ).toInvertedValue()
        );

        // Limits the motor's current
        wristMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(wristConfig.Limits.motorCurrentLimit);

        wristMotorConfig.Slot0
                .withKP(wristConfig.ControlConstants.motorPidfCoefficients.getP())
                .withKI(wristConfig.ControlConstants.motorPidfCoefficients.getI())
                .withKD(wristConfig.ControlConstants.motorPidfCoefficients.getD())
                .withKS(wristConfig.ControlConstants.motorSvagGains.getS())
                .withKV(wristConfig.ControlConstants.motorSvagGains.getV())
                .withKA(wristConfig.ControlConstants.motorSvagGains.getA())
                .withKG(wristConfig.ControlConstants.motorSvagGains.getG());

        // Sets the time the motor should take to get to the desired speed / position
        wristMotorConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(wristConfig.ControlConstants.motorRampRate);

        // Clear all sticky faults when initializing
        wristMotorController.clearStickyFaults();

        // Apply the configuration
        wristMotorController.getConfigurator().apply(wristMotorConfig);

        // Apply throughbore value to motor's encoder
        wristMotorController.setPosition(
                Rotations.of(wristEncoder.get() - wristConfig.PhysicalDescription.encoderOffset.in(Rotations)));
    }

    public record DeviceIdentifiers(NumericId wristMotorID, NumericId encoderID) {}
    public record DeviceProperties(MotorProperties wristMotorProperties) {}
    public record DeviceLimits(
            Current motorCurrentLimit, Angle minimumAngleLimit,
            Angle maximumAngleLimit, AngularVelocity maximumAngularVelocity) {}
    public record DeviceConventions(RotationalDirection motorRotationalPositiveDirection) {}
    public record PhysicalDescription(GearRatio motorsGearRatio, Angle encoderOffset) {}
    public record ControlConstants(PidfCoefficients motorPidfCoefficients, SvagGains motorSvagGains,
                                   NeutralModeValue motorNeutralMode, Time motorRampRate) {}
    public record Config(
            DeviceIdentifiers Identifiers, DeviceProperties Properties, DeviceLimits Limits,
            DeviceConventions Conventions, PhysicalDescription PhysicalDescription, ControlConstants ControlConstants
    ) {}
}
