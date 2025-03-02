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
    private SparkMax leadingWristMotorController;
    private SparkMax followerWristMotorController;
    private SparkClosedLoopController leadingWristClosedLoopController;
    private DutyCycleEncoder wristEncoder;
    private Config wristConfig;
    private Supplier<Boolean> isElevatorAngleDown;

    public WristController(Config wristConfig, Supplier<Boolean> isElevatorAngleDown) {
        this.wristConfig = wristConfig;
        this.isElevatorAngleDown = isElevatorAngleDown;

        // wrist encoder --> throughbore (roboRIO connected)
        wristEncoder = new DutyCycleEncoder(wristConfig.Identifiers.encoderID.getId());

        // leading motor --> right motor
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
        return Angle.ofBaseUnits(
                wristConfig.PhysicalDescription.motorsGearRatio.apply(getMotorEncoderAngle().in(Rotations)),
                Rotations);
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
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        // Sets leading motor to brake and decides weather to invert the motor or not
        leaderConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(
                wristConfig.PhysicalDescription.motorsGearRatio.transformRotation(
                        wristConfig.Conventions.motorsRotationalPositiveDirection
                ).differs(wristConfig.Properties.wristMotorProperties.getPositiveDirection())
        );

        // Limits the motor's current
        leaderConfig.smartCurrentLimit(
                (int) wristConfig.Limits.motorsCurrentLimit.in(Amps), // Stall limit
                (int) wristConfig.Limits.motorsCurrentLimit.in(Amps), // Free limit
                (int) wristConfig.Limits.maximumAngularVelocity.in(RotationsPerSecond) * 60 // Limit RPM
        );
        //leaderConfig.smartCurrentLimit((int) wristConfig.Limits.motorsCurrentLimit.in(Amps)); // Limited RPMs above to test

        // Sets all PIDF coefficients to the motor closed loop controller
        leaderConfig.closedLoop.pidf(
                wristConfig.ControlConstants.motorsPidfCoefficients.getP(),
                wristConfig.ControlConstants.motorsPidfCoefficients.getI(),
                wristConfig.ControlConstants.motorsPidfCoefficients.getD(),
                wristConfig.ControlConstants.motorsPidfCoefficients.getF()
        );

        // Compensate voltage by applying the summary of the gains S & G
        // S gain: minimum voltage to move.      G gain: minimum voltage to overcome the gravity.
        leaderConfig.voltageCompensation(
                wristConfig.ControlConstants.motorsSvagGains.getS() +
                        wristConfig.ControlConstants.motorsSvagGains.getG()
        );

        // Sets the time the motor should take to get to the desired speed / position
        leaderConfig.closedLoopRampRate(wristConfig.ControlConstants.motorsRampRate.in(Seconds));

        // Tells the follower motor to do everything the leading motor does
        followerConfig.follow(leadingWristMotorController.getDeviceId());

        // Removes all sticky faults when initializing
        leadingWristMotorController.clearFaults();
        followerWristMotorController.clearFaults();

        // Sets the motor's encoder reading to the throughbore encoder's reading and applies offset
        leadingWristMotorController.getEncoder().setPosition(
                wristEncoder.get() - wristConfig.PhysicalDescription.encoderOffset.in(Rotations));

        // Apply configurations
        leadingWristMotorController.configure(
                leaderConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        followerWristMotorController.configure(
                followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
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
