package net.tecdroid.subsystems.elevatorjoint;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.tecdroid.util.*;

public class ElevatorJoint extends SubsystemBase {
    private final TalonFX mLeadingMotorController; // right motor
    private final TalonFX mFollowingMotorController; // left motor
    private final DutyCycleEncoder mTroughBoreController;
    private final Config mElevatorAngleConfig;

    public ElevatorJoint(Config mElevatorAngleConfig) {
        this.mElevatorAngleConfig = mElevatorAngleConfig;

        //Both leading and follower motor are initialized here
        mLeadingMotorController = new TalonFX(mElevatorAngleConfig.Identifiers.leadingMotorId.getId());
        mFollowingMotorController = new TalonFX(mElevatorAngleConfig.Identifiers.followerMotorId.getId());
        mTroughBoreController = new DutyCycleEncoder(mElevatorAngleConfig.Identifiers.troughBorePort.getId());

        //Configures the elevator angle motors'
        motorsInterface();

        // ////////////// //
        // Elevator Angle //
        // ////////////// //
    }

    public boolean isAngleWithinRange(Angle angle) {
        //Ensures that the desired elevator´s angle is within the allowed range
        //gte = greater than or equal value            lte = less than or equal value
        return angle.gte(mElevatorAngleConfig.Limits.minimunElevatorAngle) && angle.lte(mElevatorAngleConfig.Limits.maximunElevatorAngle);
    }

    private Angle getElevatorMotorsAngle() {
        return Rotations.of(mLeadingMotorController.getPosition().getValueAsDouble());
    }

    public Angle getElevatorAngle() {
        return Rotations.of(
                mElevatorAngleConfig.PhysicalDescription.elevatorAngleMotorGR.apply(
                        getElevatorMotorsAngle().in(Rotations)
                )
        );
    }

    private Angle getAbsouluteEncoderAngle() {
        return Rotations.of(mTroughBoreController.get() - mElevatorAngleConfig.PhysicalDescription.encoderOffset.in(Rotations));
    }

    private void goToPosition(Angle position) {
        if (isAngleWithinRange(position)) {
            // Know if we are going to have gravity to change the slot
            int slot = position.in(Degrees)
                    > mElevatorAngleConfig.Limits.gravityPointAngle.in(Degrees) ? 0 : 1;
            // create a Motion Magic request, voltage output
            final MotionMagicVoltage m_request = new MotionMagicVoltage(
                    Rotations.of(
                            mElevatorAngleConfig.PhysicalDescription.elevatorAngleMotorGR
                                    .unapply(position.in(Rotations)))
            ).withSlot(slot);

            // set motor position to target angle
            mLeadingMotorController.setControl(m_request);
        }
    }

    public Command goToPositionCMD(Angle position) {
        return runOnce(
                () -> {
                    goToPosition(position);
                });
    }

    public void setVoltage(double voltage) {
        mLeadingMotorController.set(voltage);
    }

    public void stopMotors() {
        setVoltage(0.0);
    }

    public void motorsInterface() {
        //Sets elevator´s angle motors configurations

        //  Initialize configuration object
        TalonFXConfiguration motorsConfig = new TalonFXConfiguration();

        //Sets the motor to brake and defines whether to invert it or not
        motorsConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake).withInverted(
                mElevatorAngleConfig.PhysicalDescription.elevatorAngleMotorGR.transformRotation(
                        mElevatorAngleConfig.Conventions.elevatorAngleMotorsPositiveDirection
                ).toInvertedValue()
        );

        //Limit the motor´s current
        motorsConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(mElevatorAngleConfig.Limits.motorsCurrent);

        //Get the PID´s and SVAG´s values
        // The slot 0 is when we have no gravity
        motorsConfig.Slot0
                .withKP(mElevatorAngleConfig.ControlConstants.elevatorAnglePidfCoefficients.getP())
                .withKI(mElevatorAngleConfig.ControlConstants.elevatorAnglePidfCoefficients.getI())
                .withKD(mElevatorAngleConfig.ControlConstants.elevatorAnglePidfCoefficients.getD())
                .withKS(mElevatorAngleConfig.ControlConstants.elevatorAngleNoGravitySvagGains.getS())
                .withKV(mElevatorAngleConfig.ControlConstants.elevatorAngleNoGravitySvagGains.getV())
                .withKA(mElevatorAngleConfig.ControlConstants.elevatorAngleNoGravitySvagGains.getA())
                .withKG(mElevatorAngleConfig.ControlConstants.elevatorAngleNoGravitySvagGains.getG());

        // The slot 1 is when we have gravity
        motorsConfig.Slot1
                .withKP(mElevatorAngleConfig.ControlConstants.elevatorAnglePidfCoefficients.getP())
                .withKI(mElevatorAngleConfig.ControlConstants.elevatorAnglePidfCoefficients.getI())
                .withKD(mElevatorAngleConfig.ControlConstants.elevatorAnglePidfCoefficients.getD())
                .withKS(mElevatorAngleConfig.ControlConstants.elevatorAngleGravitySvagGains.getS())
                .withKV(mElevatorAngleConfig.ControlConstants.elevatorAngleGravitySvagGains.getV())
                .withKA(mElevatorAngleConfig.ControlConstants.elevatorAngleGravitySvagGains.getA())
                .withKG(mElevatorAngleConfig.ControlConstants.elevatorAngleGravitySvagGains.getG());

        //Get the Motion Magic values
        motorsConfig.MotionMagic
                .withMotionMagicCruiseVelocity(mElevatorAngleConfig.ControlConstants.elevatorAngleMotionMagicCoefficients.getMotionMagicCruiseVelocity())
                .withMotionMagicAcceleration(mElevatorAngleConfig.ControlConstants.elevatorAngleMotionMagicCoefficients.getMotionMagicAcceleration())
                .withMotionMagicJerk(mElevatorAngleConfig.ControlConstants.elevatorAngleMotionMagicCoefficients.getMotionMagicJerk());

        // Sets the time the motor should take to get to the desired speed / position
        motorsConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(mElevatorAngleConfig.ControlConstants.elevatorAngleRampRate);

        // Clear all sticky faults when initializing
        mLeadingMotorController.clearStickyFaults();
        mFollowingMotorController.clearStickyFaults();

        // Apply the motor´s configuration
        mLeadingMotorController.getConfigurator().apply(motorsConfig);
        mFollowingMotorController.getConfigurator().apply(motorsConfig);

        // Setting motors reading to the trough bore with the specified zero position
        mLeadingMotorController.setPosition(
                mElevatorAngleConfig.PhysicalDescription.elevatorAngleMotorGR.unapply(
                        getAbsouluteEncoderAngle()));

        mFollowingMotorController.setPosition(
                mElevatorAngleConfig.PhysicalDescription.elevatorAngleMotorGR.unapply(
                        getAbsouluteEncoderAngle()));

        // Set follower motor
        mFollowingMotorController.setControl(new Follower(mLeadingMotorController.getDeviceID(), true));
    }

    public record DeviceIdentifiers(NumericId leadingMotorId, NumericId followerMotorId, NumericId troughBorePort) {}
    public record DeviceProperties(MotorProperties motorsProperties) {}
    public record DeviceLimits(
            Current motorsCurrent, Angle minimunElevatorAngle,
            Angle maximunElevatorAngle, Angle gravityPointAngle) {}
    public record DeviceConventions(RotationalDirection elevatorAngleMotorsPositiveDirection) {}
    public record PhysicalDescription(GearRatio elevatorAngleMotorGR, Angle encoderOffset) {}
    public record ControlConstants(
            PidfCoefficients elevatorAnglePidfCoefficients,
            SvagGains elevatorAngleNoGravitySvagGains,
            SvagGains elevatorAngleGravitySvagGains,
            MotionMagicSettings elevatorAngleMotionMagicCoefficients,
            Time elevatorAngleRampRate) {}
    public record Config(
            DeviceIdentifiers Identifiers, DeviceProperties Properties,
            DeviceLimits Limits, DeviceConventions Conventions,
            PhysicalDescription PhysicalDescription, ControlConstants ControlConstants) {}
}