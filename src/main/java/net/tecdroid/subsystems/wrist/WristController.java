package net.tecdroid.subsystems.wrist;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import net.tecdroid.util.*;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class WristController {
    private final SparkMax leadingWristMotorController;
    private final SparkMax followerWristMotorController;
    private final SparkClosedLoopController leadingWristClosedLoopController;
    private final DutyCycleEncoder wristEncoder;
    private final Config wristConfig;
    private final Supplier<Boolean> isElevatorAngleDown;

    public WristController(Config wristConfig, Supplier<Boolean> isElevatorAngleDown) {
        this.wristConfig = wristConfig;
        this.isElevatorAngleDown = isElevatorAngleDown;

        // wrist encoder --> throughbore (roboRIO connected)
        wristEncoder = new DutyCycleEncoder(wristConfig.Identifiers.encoderID.getId());

        // leading motor --> left motor
        leadingWristMotorController = new SparkMax(wristConfig.Identifiers.leftMotorID.getId(),
                                                    SparkLowLevel.MotorType.kBrushless);
        // follower motor --> right motor
        followerWristMotorController = new SparkMax(wristConfig.Identifiers.rightMotorID.getId(),
                                                    SparkLowLevel.MotorType.kBrushless);
        // Configure motors
        motorsInterface();

        leadingWristClosedLoopController = leadingWristMotorController.getClosedLoopController();
    }

    public Angle getWristAngle() {
        /* Gets the leading motor's encoder reading and applies the gear ratio to obtain the wrist angle */
        return Rotations.of(wristConfig.PhysicalDescription.motorsGearRatio.apply(getMotorEncoderAngle().in(Rotations)));
    }

    private Angle getMotorEncoderAngle() {
        /* Returns the leading motor's encoder reading as an Angle object from rotations */
        return Rotations.of(leadingWristMotorController.getEncoder().getPosition());
    }

    public void setWristAngle(Angle angle) {
        /* Takes the desired wrist angle, converts it to motor rotations and applies it */
        // Checks that the desired angle is within allowed range and that elevator angle is not down.
        // Always allows to move the wrist to {insert value} degrees or more.
        if (isAngleWithinRange(angle) &&
                (!isElevatorAngleDown.get() || angle.gte(Degrees.of(0.0)))) {
            leadingWristClosedLoopController.setReference(
                    wristConfig.PhysicalDescription.motorsGearRatio.unapply(angle.in(Rotations)),
                    SparkBase.ControlType.kPosition);
        }
    }

    public boolean isAngleWithinRange(Angle angle) {
        /* Makes sure that the desired wrist angle is within the allowed range */
        // gte() == greater than or equal to       lte() == less than or equal to
        return angle.gte(wristConfig.Limits.minimumAngleLimit) && angle.lte(wristConfig.Limits.maximumAngleLimit);
    }

    // TODO: GET THROUGHBORE OFFSET

    // ///////////// //
    // Configuration //
    // ///////////// //

    public void motorsInterface() {
        /* Sets all motor's configurations */

        // Initialize config objects
        SparkMaxConfig motorsConfig = new SparkMaxConfig();

        // Sets leading motor to brake and decides whether to invert the motor or not
        motorsConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(
                wristConfig.PhysicalDescription.motorsGearRatio.transformRotation(
                        wristConfig.Conventions.motorsRotationalPositiveDirection
                ).differs(wristConfig.Properties.wristMotorProperties.getPositiveDirection())
        );

        // Limits the motor's current
        motorsConfig.smartCurrentLimit(
                (int) wristConfig.Limits.motorsCurrentLimit.in(Amps), // Stall limit
                (int) wristConfig.Limits.motorsCurrentLimit.in(Amps), // Free limit
                (int) wristConfig.Limits.maximumAngularVelocity.in(RotationsPerSecond) * 60 // Limit RPM
        );
        //leaderConfig.smartCurrentLimit((int) wristConfig.Limits.motorsCurrentLimit.in(Amps)); // Limited RPMs above to test

        // Sets all PIDF coefficients to the motor closed loop controller
        motorsConfig.closedLoop.pidf(
                wristConfig.ControlConstants.motorsPidfCoefficients.getP(),
                wristConfig.ControlConstants.motorsPidfCoefficients.getI(),
                wristConfig.ControlConstants.motorsPidfCoefficients.getD(),
                wristConfig.ControlConstants.motorsPidfCoefficients.getF()
        );

        // Compensate voltage by applying the summary of the gains S & G
        // S gain: minimum voltage to move.      G gain: minimum voltage to overcome the gravity.
        motorsConfig.voltageCompensation(
                wristConfig.ControlConstants.motorsSvagGains.getS() +
                        wristConfig.ControlConstants.motorsSvagGains.getG()
        );

        // Sets the time the motor should take to get to the desired speed / position
        motorsConfig.closedLoopRampRate(wristConfig.ControlConstants.motorsRampRate.in(Seconds));

        // Sets the motor's encoder reading to the throughbore encoder's reading and applies offset
        leadingWristMotorController.getEncoder().setPosition(
                wristEncoder.get() - wristConfig.PhysicalDescription.encoderOffset.in(Rotations));

        // Removes all previous sticky faults
        leadingWristMotorController.clearFaults();
        followerWristMotorController.clearFaults();

        // Apply configurations
        leadingWristMotorController.configure(
                motorsConfig,
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        followerWristMotorController.configure(
                motorsConfig,
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Sets the motor to follower mode. kNoResetSafeParameters to avoid overwriting previous set of configs
        followerWristMotorController.configure(
                motorsConfig.follow(leadingWristMotorController.getDeviceId()),
                SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters
        );
    }

    public record DeviceIdentifiers(NumericId leftMotorID, NumericId rightMotorID, NumericId encoderID) {}
    public record DeviceProperties(MotorProperties wristMotorProperties) {}
    public record DeviceLimits(
            Current motorsCurrentLimit, Angle minimumAngleLimit,
            Angle maximumAngleLimit, AngularVelocity maximumAngularVelocity) {}
    public record DeviceConventions(RotationalDirection motorsRotationalPositiveDirection) {}
    public record PhysicalDescription(GearRatio motorsGearRatio, Angle encoderOffset) {}
    public record ControlConstants(PidfCoefficients motorsPidfCoefficients, SvagGains motorsSvagGains, Time motorsRampRate) {}
    public record Config(
            DeviceIdentifiers Identifiers, DeviceProperties Properties, DeviceLimits Limits,
            DeviceConventions Conventions, PhysicalDescription PhysicalDescription, ControlConstants ControlConstants
    ) {}
}
