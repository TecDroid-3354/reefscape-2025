package net.tecdroid.subsystems.wrist;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import net.tecdroid.util.*;

import static edu.wpi.first.units.Units.*;

public class WristController {
    // leading == left NEO
    private SparkMax leadingWristMotorController;
    // follower == right NEO
    private SparkMax followerWristMotorController;
    private SparkClosedLoopController leadingWristClosedLoopController;
    private DutyCycleEncoder wristEncoder;
    private Config wristConfig;

    public WristController(Config config) {
        // leadingWristMotor --> leftMotor
        wristConfig = config;
        wristEncoder = new DutyCycleEncoder(config.Identifiers.encoderID.getId());
        leadingWristMotorController = new SparkMax(config.Identifiers.leftMotorID.getId(), SparkLowLevel.MotorType.kBrushless);
        followerWristMotorController = new SparkMax(config.Identifiers.rightMotorID.getId(), SparkLowLevel.MotorType.kBrushless);
        motorsInterface();

        leadingWristMotorController.getAlternateEncoder().setPosition(
                leadingWristMotorController.getAlternateEncoder().getPosition() +
                        config.PhysicalDescription.encoderOffset.in(Rotations)
        );
        leadingWristClosedLoopController = leadingWristMotorController.getClosedLoopController();
    }

    public Angle getWristAngle() {
        return Angle.ofBaseUnits(leadingWristMotorController.getEncoder().getPosition(), Rotations);
    }

    public void setWristAngle(Angle angle) {
        if (isAngleWithinRange(angle)) {
            leadingWristClosedLoopController.setReference(
                    angle.in(Rotations),
                    SparkBase.ControlType.kPosition);
        }
    }

    public boolean isAngleWithinRange(Angle angle) {
        // gte() == greater than or equal       lte() == less than or equal
        return angle.gte(wristConfig.Limits.minimumAngleLimit) && angle.lte(wristConfig.Limits.maximumAngleLimit);
    }

    // TODO: GET THROUGHBORE OFFSET
    // TODO: WHEN SLIDER ANGLE IS DOWN, WRIST MUST BE UP

    public void motorsInterface() {
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leaderConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(
                wristConfig.Conventions.motorsRotationalPositiveDirection !=
                wristConfig.Properties.wristMotorProperties.getPositiveDirection()
        );

        leaderConfig.smartCurrentLimit((int) wristConfig.Limits.motorsCurrentLimit.in(Amps));

        leaderConfig.closedLoop.pidf(
                wristConfig.ControlConstants.motorsPidfCoefficients.getP(),
                wristConfig.ControlConstants.motorsPidfCoefficients.getI(),
                wristConfig.ControlConstants.motorsPidfCoefficients.getD(),
                wristConfig.ControlConstants.motorsPidfCoefficients.getF()
        );

        // TODO: PONER RAMPRATE EN EL CONFIG
        leaderConfig.closedLoopRampRate(0.1);

        followerConfig.follow(leadingWristMotorController.getDeviceId());

        leadingWristMotorController.clearFaults();
        followerWristMotorController.clearFaults();

        leadingWristMotorController.getEncoder().setPosition(
                wristEncoder.get() - wristConfig.PhysicalDescription.encoderOffset.in(Rotations));

        leadingWristMotorController.configure(
                leaderConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        followerWristMotorController.configure(
                followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }


    public record DeviceIdentifiers(NumericId leftMotorID, NumericId rightMotorID, NumericId encoderID) {}
    public record DeviceProperties(MotorProperties wristMotorProperties) {}
    public record DeviceLimits(Current motorsCurrentLimit, Angle minimumAngleLimit, Angle maximumAngleLimit) {}
    public record DeviceConventions(RotationalDirection motorsRotationalPositiveDirection) {}
    public record PhysicalDescription(GearRatio motorsGearRatio, Angle encoderOffset) {} // TODO: CREATE OFFSET CLASS ?
    public record ControlConstants(PidfCoefficients motorsPidfCoefficients, SvagGains motorsSvagGains) {}
    public record Config(
            DeviceIdentifiers Identifiers, DeviceProperties Properties, DeviceLimits Limits,
            DeviceConventions Conventions, PhysicalDescription PhysicalDescription, ControlConstants ControlConstants
    ) {}
}
