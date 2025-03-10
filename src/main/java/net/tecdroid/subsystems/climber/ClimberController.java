package net.tecdroid.subsystems.climber;

import static net.tecdroid.subsystems.climber.ClimberConfiguration.climberConfiguration;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import net.tecdroid.util.*;

public class ClimberController extends SubsystemBase {
    private TalonFX leadingClimberMotorController;
    private TalonFX followerClimberMotorController;
    private DutyCycleEncoder climberEncoder;
    public final Config climberConfig = climberConfiguration;

    // System identification

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle m_angle = Radians.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

    // Constructor
    public ClimberController() {
        // Troughbore encoder (roboRIO connected)
        climberEncoder = new DutyCycleEncoder(climberConfig.Identifiers.encoderPort.getId());

        // Leading motor -> Left motor
        leadingClimberMotorController = new TalonFX(climberConfig.Identifiers.leftClimberMotorID.getId());

        // Follower motor -> right motor
        followerClimberMotorController = new TalonFX(climberConfig.Identifiers.rightClimberMotorID.getId());

        // Configure motors
        MotorsInterface();
    }

    SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    this::setVoltage,
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor("climber-svga-gains")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                leadingClimberMotorController.get() * RobotController.getBatteryVoltage(), Volts))
                                .angularPosition(m_angle.mut_replace(leadingClimberMotorController.getPosition().getValueAsDouble(), Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(leadingClimberMotorController.getVelocity().getValueAsDouble(), RotationsPerSecond));
                    },
                    this)
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public boolean isInLimit(Voltage voltage) {
        return (getClimberAngle().in(Rotations) < climberConfig.Limits.minimumClimberAngle.in(Rotations) && voltage.in(Volts) < 0.0) ||
                (getClimberAngle().in(Rotations) > climberConfig.Limits.maximumClimberAngle.in(Rotations) && voltage.in(Volts) > 0.0);
    }

    public void setVoltage(Voltage voltage) {
        if (!isInLimit(voltage)) {
            leadingClimberMotorController.setControl(new VoltageOut(voltage));
        } else {
            stopMotors();
        }
    }

    public void stopMotors() {
        setVoltage(Volts.of(0.0));
    }

    public Command goToPositionCMD(Angle requestedAngle) {
        return runOnce(
                () -> {
                    goToPosition(requestedAngle);
                });
    }

    private void goToPosition(Angle requestedAngle) {
        if (isAngleWithinRange(requestedAngle)) {
            // pre-process position (inches to rotations)
            Angle requestedRotations = Rotations.of(climberConfig.PhysicalDescription.climberGearRatio.unapply(
                    requestedAngle.in(Rotations)
            ));

            // create a Motion Magic request, voltage output
            final MotionMagicVoltage m_request = new MotionMagicVoltage(requestedRotations);

            // set target position to 100 rotations
            leadingClimberMotorController.setControl(m_request);
        }
    }

    public Angle getClimberAngle() {
        /* Applies the gear ratio to the motor's encoder reading to obtain the climber angle position */
        return climberConfig.PhysicalDescription.climberGearRatio.apply(getMotorEncoderPosition());
    }

    private Angle getMotorEncoderPosition() {
        /* Returns the current angle reading of the motor */
        return leadingClimberMotorController.getPosition().getValue();
    }

    private Angle getAbsoluteEncoderPosition() {
        return Rotations.of(climberEncoder.get() - climberConfig.PhysicalDescription.encoderOffSet.in(Rotations));
    }

    private boolean isAngleWithinRange(Angle angle) {
        /* Checks that the requested angle is within the climber's limits */
        return angle.gte(climberConfig.Limits.minimumClimberAngle) && angle.lte(climberConfig.Limits.maximumClimberAngle);
    }

    public void publishToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        tab.addDouble("Climber rotations", () -> getClimberAngle().in(Rotations));
        tab.addDouble("Absolute Encoder Rotations", () -> getAbsoluteEncoderPosition().in(Rotations));
        tab.addDouble("Motor Rotations", () -> getMotorEncoderPosition().in(Rotations));

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

        // set slot 0 gains
        var slot0Configs = motorsConfig.Slot0;
        slot0Configs.kS = climberConfig.coefficients.svagGains.getS();
        slot0Configs.kV = climberConfig.coefficients.svagGains.getV();
        slot0Configs.kA = climberConfig.coefficients.svagGains.getA();
        slot0Configs.kP = climberConfig.coefficients.pidfCoefficients.getP();
        slot0Configs.kI = climberConfig.coefficients.pidfCoefficients.getI();
        slot0Configs.kD = climberConfig.coefficients.pidfCoefficients.getD();

        // Apply Motion Magic control constants
        motorsConfig.MotionMagic
                .withMotionMagicCruiseVelocity(climberConfig.ControlConstants.climberMotionTargets.getCruiseVelocity())
                .withMotionMagicAcceleration(climberConfig.ControlConstants.climberMotionTargets.getAcceleration())
                .withMotionMagicJerk(climberConfig.ControlConstants.climberMotionTargets.getJerk());

        // Motors will take 0.1 seconds to achieve the desired speed
        motorsConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(climberConfig.ControlConstants.climberRampRate);

        // Clear sticky faults previous to configuration
        leadingClimberMotorController.clearStickyFaults();
        followerClimberMotorController.clearStickyFaults();

        // Apply configuration
        leadingClimberMotorController.getConfigurator().apply(motorsConfig);
        followerClimberMotorController.getConfigurator().apply(motorsConfig);

        // Set leading motor position to throughbore reading
        leadingClimberMotorController.setPosition(
                Rotations.of(climberConfig.PhysicalDescription.climberGearRatio.unapply(getAbsoluteEncoderPosition().in(Rotations))));

        // Set the following motor to follow the leader
        followerClimberMotorController.setControl(new Follower(leadingClimberMotorController.getDeviceID(), false));
    }

    public record DeviceIdentifiers(NumericId encoderPort, NumericId leftClimberMotorID, NumericId rightClimberMotorID) {}
    public record DeviceProperties(MotorProperties climberMotorsProperties) {}
    public record DeviceLimits(
            Current climberMotorsCurrentLimit, Angle minimumClimberAngle, Angle maximumClimberAngle) {}
    public record DeviceConventions(RotationalDirection climberRotationalPositiveDirection) {}
    public record PhysicalDescription(GearRatio climberGearRatio, Angle encoderOffSet) {}
    public record ControlConstants(MotionTargets climberMotionTargets,
                                   Time climberRampRate, NeutralModeValue motorsNeutralMode) {}
    public record Coefficients(PidfCoefficients pidfCoefficients, SvagGains svagGains) {}
    public record Config(DeviceIdentifiers Identifiers, DeviceProperties Properties, DeviceLimits Limits,
                         DeviceConventions Conventions, PhysicalDescription PhysicalDescription,
                         ControlConstants ControlConstants, Coefficients coefficients) {}
}

