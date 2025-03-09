package net.tecdroid.subsystems.climber;

import static net.tecdroid.subsystems.climber.ClimberConfiguration.climberConfiguration;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.units.measure.*;
import net.tecdroid.util.*;

public class ClimberController {
    private final TalonFX leadingClimberMotorController;
    private final TalonFX followerClimberMotorController;
    private final DutyCycleEncoder climberEncoder;
    private final Config climberConfig = climberConfiguration;


    public ClimberController(Config climberConfig) {
        // Troughbore encoder (roboRIO connected)
        climberEncoder = new DutyCycleEncoder(climberConfig.Identifiers.encoderPort.getId());

        // Leading motor -> Left motor
        leadingClimberMotorController = new TalonFX(climberConfig.Identifiers.leftClimberMotorID.getId());

        // Follower motor -> right motor
        followerClimberMotorController = new TalonFX(climberConfig.Identifiers.rightClimberMotorID.getId());

        // Configure motors
        MotorsInterface();
    }

    public void setClimberPosition(Angle angle) {
        /* Takes the desired climber angle and unapplies the gear ratio to obtain the motor's desired  rotations*/
        final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
        if(isAngleWithinRange(angle)) {
            leadingClimberMotorController.setControl(positionRequest.withPosition(
                climberConfig.PhysicalDescription.climberGearRatio.unapply(angle)
            ));
        }
    }

    public Angle getClimberPosition() {
        /* Applies the gear ratio to the motor's encoder reading to obtain the climber angle position */
        return climberConfig.PhysicalDescription.climberGearRatio.apply(getMotorEncoderPosition());
    }

    private Angle getMotorEncoderPosition() {
        /* Returns the current angle reading of the motor */
        return leadingClimberMotorController.getPosition().getValue();
    }

    private boolean isAngleWithinRange(Angle angle) {
        /* Checks that the requested angle is within the climber's limits */
        return angle.gte(climberConfig.Limits.minimumClimberAngle) && angle.lte(climberConfig.Limits.maximumClimberAngle);
    }

    // ///////////// //
    // Configuration //
    // ///////////// //

    private void MotorsInterface() {
        // Initializes the configuration object
        TalonFXConfiguration motorsConfig = new TalonFXConfiguration();

        // Sets the neutral mode value and decides whether to invert the motors or not
        motorsConfig.MotorOutput.withNeutralMode(climberConfig.ControlConstants.motorsNeutralMode)
            .withInverted(
                climberConfig.PhysicalDescription.climberGearRatio.transformRotation(
                    climberConfig.Conventions.climberRotationalPositiveDirection).toInvertedValue());

        // Set the current limit. In case of krakens, 40 Amps
        motorsConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(climberConfig.Limits.climberMotorsCurrentLimit);

        // Apply Motion Magic control constants
        motorsConfig.MotionMagic
            .withMotionMagicCruiseVelocity(climberConfig.ControlConstants.climberMotionMagicTargets.getCruiseVelocity())
            .withMotionMagicAcceleration(climberConfig.ControlConstants.climberMotionMagicTargets.getAcceleration())
            .withMotionMagicJerk(climberConfig.ControlConstants.climberMotionMagicTargets.getJerk());

        // Motors will take 0.1 seconds to achieve the desired speed
        motorsConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(climberConfig.ControlConstants.climberRampRate);

        // Clear sticky faults previous to configuration
        leadingClimberMotorController.clearStickyFaults();
        followerClimberMotorController.clearStickyFaults();

        // Apply configuration
        leadingClimberMotorController.getConfigurator().apply(motorsConfig);
        followerClimberMotorController.getConfigurator().apply(motorsConfig);

        // Set leading motor position to throughbore reading minus the offset to get the desired zero position
        leadingClimberMotorController.setPosition(
            Rotations.of(climberEncoder.get() - climberConfig.PhysicalDescription.encoderOffSet.in(Rotations)));

        // Set the following motor to follow the leader motor in the same direction
        followerClimberMotorController.setControl(new Follower(leadingClimberMotorController.getDeviceID(), false));
    }

    public record DeviceIdentifiers(NumericId encoderPort, NumericId leftClimberMotorID, NumericId rightClimberMotorID) {}
    public record DeviceProperties(MotorProperties climberMotorsProperties) {}
    public record DeviceLimits(
        Current climberMotorsCurrentLimit, Angle minimumClimberAngle, Angle maximumClimberAngle) {}
    public record DeviceConventions(RotationalDirection climberRotationalPositiveDirection) {}
    public record PhysicalDescription(GearRatio climberGearRatio, Angle encoderOffSet) {}
    public record ControlConstants(MotionMagicTargets climberMotionMagicTargets,
                                   Time climberRampRate, NeutralModeValue motorsNeutralMode) {}
    public record Config(DeviceIdentifiers Identifiers, DeviceProperties Properties, DeviceLimits Limits,
                         DeviceConventions Conventions, PhysicalDescription PhysicalDescription, ControlConstants ControlConstants) {}
}
