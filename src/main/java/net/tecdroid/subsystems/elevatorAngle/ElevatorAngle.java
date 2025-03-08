package net.tecdroid.subsystems.elevatorAngle;

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

public class ElevatorAngle extends SubsystemBase {
    private final TalonFX mLeadingMotorController; // right motor
    private final TalonFX mFollowingMotorController; // left motor
    private final DutyCycleEncoder mTroughBoreController;
    private final Config mElevatorAngleConfig;

    public ElevatorAngle(Config mElevatorAngleConfig) {
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

    public Command goToPosition(Angle position) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        if (!isAngleWithinRange(position)) {
            return null;
        }
        return runOnce(
                () -> {
                    // create a Motion Magic request, voltage output
                    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
                    Angle requestedPosition = Rotations.of(
                            mElevatorAngleConfig.PhysicalDescription.elevatorAngleMotorGR
                                    .unapply(position.in(Rotations)));

                    // set motor position to target angle
                    mLeadingMotorController.setControl(m_request.withPosition(requestedPosition.in(Rotations)));
                });
    }
    public boolean isAngleWithinRange(Angle angle) {
        //Ensures that the desired elevator´s angle is within the allowed range
        //gte = greater than or equal value            lte = less than or equal value
        return angle.gte(mElevatorAngleConfig.Limits.minimunElevatorAngle) && angle.lte(mElevatorAngleConfig.Limits.maximunElevatorAngle);
    }

    public Angle getElevatorAngle() {
        /* Applies the gear ratio to the motor's encoder reading to obtain the elevator angle position */
        return mElevatorAngleConfig.PhysicalDescription.elevatorAngleMotorGR.apply(
               getElevatorMotorsAngle()
        );
    }

    private Angle getElevatorMotorsAngle() {
        return Rotations.of(mLeadingMotorController.getPosition().getValueAsDouble());
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
        motorsConfig.Slot0
                .withKP(mElevatorAngleConfig.ControlConstants.elevatorAnglePidfCoefficients.getP())
                .withKI(mElevatorAngleConfig.ControlConstants.elevatorAnglePidfCoefficients.getI())
                .withKD(mElevatorAngleConfig.ControlConstants.elevatorAnglePidfCoefficients.getD())
                .withKS(mElevatorAngleConfig.ControlConstants.elevatorAngleSvagGains.getS())
                .withKV(mElevatorAngleConfig.ControlConstants.elevatorAngleSvagGains.getV())
                .withKA(mElevatorAngleConfig.ControlConstants.elevatorAngleSvagGains.getA())
                .withKG(mElevatorAngleConfig.ControlConstants.elevatorAngleSvagGains.getG());

        //Get the Motion Magic values
        motorsConfig.MotionMagic
                .withMotionMagicCruiseVelocity(mElevatorAngleConfig.ControlConstants.elevatorAngleMotionMagicCoefficients.getCruiseVelocity())
                .withMotionMagicAcceleration(mElevatorAngleConfig.ControlConstants.elevatorAngleMotionMagicCoefficients.getAcceleration())
                .withMotionMagicJerk(mElevatorAngleConfig.ControlConstants.elevatorAngleMotionMagicCoefficients.getJerk());

        // Sets the time the motor should take to get to the desired speed / position
        motorsConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(mElevatorAngleConfig.ControlConstants.elevatorAngleRampRate);

        //Clear all sticky faults when initializing
        mLeadingMotorController.clearStickyFaults();
        mFollowingMotorController.clearStickyFaults();

        //Apply the motor´s configuration
        mLeadingMotorController.getConfigurator().apply(motorsConfig);
        mFollowingMotorController.getConfigurator().apply(motorsConfig);

        //Setting motors reading to the troughbore with the specified zero position
        mLeadingMotorController.setPosition(
                mTroughBoreController.get() - mElevatorAngleConfig.PhysicalDescription.encoderOffset.in(Rotations));

        // Set follower motor
        mFollowingMotorController.setControl(new Follower(mLeadingMotorController.getDeviceID(), true));
    }

    public record DeviceIdentifiers(NumericId leadingMotorId, NumericId followerMotorId, NumericId troughBorePort) {}
    public record DeviceProperties(MotorProperties motorsProperties) {}
    public record DeviceLimits(
            Current motorsCurrent, Angle minimunElevatorAngle,
            Angle maximunElevatorAngle) {}
    public record DeviceConventions(RotationalDirection elevatorAngleMotorsPositiveDirection) {}
    public record PhysicalDescription(GearRatio elevatorAngleMotorGR, Angle encoderOffset) {}
    public record ControlConstants(
            PidfCoefficients elevatorAnglePidfCoefficients,
            SvagGains elevatorAngleSvagGains,
            MotionMagicCoefficients elevatorAngleMotionMagicCoefficients,
            Time elevatorAngleRampRate) {}
    public record Config(
            DeviceIdentifiers Identifiers, DeviceProperties Properties,
            DeviceLimits Limits, DeviceConventions Conventions,
            PhysicalDescription PhysicalDescription, ControlConstants ControlConstants) {}
}


