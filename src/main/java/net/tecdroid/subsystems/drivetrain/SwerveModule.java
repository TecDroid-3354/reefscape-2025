package net.tecdroid.subsystems.drivetrain;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import net.tecdroid.constants.UnitConstants;
import net.tecdroid.util.*;
import net.tecdroid.util.geometry.Wheel;

import static edu.wpi.first.units.Units.*;

public class SwerveModule implements Sendable {

    /**
     * Stores the device IDs of the module's electronic components
     * @param driveId The ID of the drive motor controller
     * @param steerId The ID of the steer motor controller
     * @param absoluteEncoderId The ID of the absolute encoder
     */
    public record LinkedIdentifiers(NumericId driveId, NumericId steerId, NumericId absoluteEncoderId) {}

    /**
     * Stores the considerations that must be taken into account due to the module's physical state
     * @param absoluteEncoderOffset The magnet offset of the absolute encoder
     * @param driveWheelPositiveDirection The direction in which the drive wheel must turn when it receives a positive input
     * @param steerWheelPositiveDirection The direction in which the steer wheel must turn when it receives a positive input
     */
    public record StateConsiderations(Angle absoluteEncoderOffset, RotationalDirection driveWheelPositiveDirection, RotationalDirection steerWheelPositiveDirection) {}

    /**
     * Stores characteristics relating to the module's physical description
     * @param offset The module's offset from the center of the drivetrain
     * @param driveGearing The gearing between the drive shaft and the wheel
     * @param steerGearing The gearing between the steer shaft and the wheel
     * @param wheel The wheel's physical description
     */
    public record PhysicalDescription(Translation2d offset, GearRatio driveGearing, GearRatio steerGearing, Wheel wheel) {}

    /**
     * Stores the control constants that will be applied to the module's motion
     * @param drivePidf The PIDF feedback coefficients used to control the driving motion
     * @param driveSvag The SVAG feedforward gains used to control the driving motion
     * @param steerPidf The PIDF feedback coefficients used to control the steering motion
     * @param steerSvag The SVAG feedforward gains used to control the steering motion
     */
    public record ControlConstants(PidfCoefficients drivePidf, SvagGains driveSvag, PidfCoefficients steerPidf, SvagGains steerSvag) {}

    /**
     * Stores limits that regulate the module's motion
     * @param driveCurrentLimit The current limit for the driving motor
     * @param steerCurrentLimit The current limit for the steering motor
     */
    public record Limits(Current driveCurrentLimit, Current steerCurrentLimit) {}

    /**
     * Stores the module's configuration parameters
     * @param identifiers The identifier config
     * @param state The state config
     * @param physical The physical config
     * @param control The control config
     * @param limits The limt config
     */
    public record Config(LinkedIdentifiers identifiers, StateConsiderations state, PhysicalDescription physical, ControlConstants control, Limits limits) {}

    private final Config config;

    private final TalonFX driveInterface;

    private final SparkMax                  steerController;
    private final RelativeEncoder           steerEncoder;
    private final SparkClosedLoopController steerClosedLoopController;

    private final CANcoder absoluteEncoder;

    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(Config config) {
        this.config = config;

        this.driveInterface = new TalonFX(config.identifiers.driveId.getId());
        this.configureDriveInterface();

        this.steerController           = new SparkMax(config.identifiers.steerId.getId(), MotorType.kBrushless);
        this.steerEncoder              = steerController.getEncoder();
        this.steerClosedLoopController = steerController.getClosedLoopController();
        this.configureSteerInterface();

        this.absoluteEncoder = new CANcoder(config.identifiers.absoluteEncoderId.getId());
        this.configureAbsoluteEncoderInterface();

    }

    public void setTargetState(SwerveModuleState newState) {
        newState.optimize(new Rotation2d((getSteerWheelAngularPosition())));
        this.targetState = newState;

        final Angle requestedAngle = newState.angle.getMeasure();
        final Angle shaftAngle = steerWheelAngularPositionToSteerShaftAngularPosition(requestedAngle);
        this.steerClosedLoopController.setReference(shaftAngle.in(Rotations), ControlType.kPosition);

        final LinearVelocity requestedVelocity = MetersPerSecond.of(newState.speedMetersPerSecond);
        final AngularVelocity shaftVelocity = driveWheelLinearVelocityToDriveShaftAngularVelocity(requestedVelocity);
        this.driveInterface.setControl(new VelocityVoltage(shaftVelocity).withSlot(0));
    }

    public void matchSteeringEncoderToAbsoluteEncoder() {
        steerEncoder.setPosition(getAbsoluteSteerShaftPosition().in(Rotations));
    }

    // ///////////////// //
    // Getters + Setters //
    // ///////////////// //

    public Angle getAbsoluteSteerWheelPosition() {
        return absoluteEncoder.getPosition().getValue();
    }

    public Angle getAbsoluteSteerShaftPosition() {
        return config.physical.steerGearing.unapply(getAbsoluteSteerWheelPosition());
    }

    public Angle getDriveShaftPosition() {
        return driveInterface.getPosition().getValue();
    }

    public Angle getDriveWheelAngularDisplacement() {
        return config.physical.driveGearing.apply(getDriveShaftPosition());
    }

    public Distance getDriveWheelLinearDisplacement() {
        return config.physical.wheel.angularDisplacementToLinearDisplacement(getDriveWheelAngularDisplacement());
    }

    public AngularVelocity getDriveShaftAngularVelocity() {
        return driveInterface.getVelocity().getValue();
    }

    public AngularVelocity getDriveWheelAngularVelocity() {
        return config.physical.driveGearing.apply(getDriveShaftAngularVelocity());
    }

    public LinearVelocity getDriveWheelLinearVelocity() {
        return config.physical.wheel.angularVelocityToLinearVelocity(getDriveWheelAngularVelocity());
    }

    public Angle getSteerShaftAngularPosition() {
        return Rotations.of(steerEncoder.getPosition());
    }

    public Angle getSteerWheelAngularPosition() {
        return config.physical.steerGearing.apply(getSteerShaftAngularPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveWheelLinearVelocity(), new Rotation2d(getSteerShaftAngularPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveWheelLinearDisplacement(), new Rotation2d(getSteerShaftAngularPosition()));
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public Translation2d getOffsetFromCenter() {
        return config.physical.offset();
    }

    // ///////////// //
    // Configuration //
    // ///////////// //

    private void configureDriveInterface() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.Audio.withBeepOnBoot(true)
                    .withBeepOnConfig(true)
                    .withAllowMusicDurDisable(true);

        driveConfig.CurrentLimits.withSupplyCurrentLimit(config.limits.driveCurrentLimit)
                            .withSupplyCurrentLimitEnable(true);

        driveConfig.Slot0.withKP(config.control.drivePidf.getP())
                         .withKI(config.control.drivePidf.getI())
                         .withKD(config.control.drivePidf.getD())
                         .withKS(config.control.driveSvag.getS())
                         .withKV(config.control.driveSvag.getV())
                         .withKA(config.control.driveSvag.getA());

        driveConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
                               .withInverted(
                                       config.physical.driveGearing.transformRotation(
                                               config.state.driveWheelPositiveDirection
                                       ).toInvertedValue()
                               );

        this.driveInterface.clearStickyFaults();
        this.driveInterface.getConfigurator().apply(driveConfig);
    }

    private void configureSteerInterface() {
        SparkMaxConfig steerConfig = new SparkMaxConfig();

        steerConfig
                .idleMode(IdleMode.kBrake)
                .inverted(
                        config.physical.steerGearing.transformRotation(
                            config.state.steerWheelPositiveDirection
                        ).differs(Motors.INSTANCE.getNeo().getRotationalConvention().positiveDirection())
                )
                .smartCurrentLimit((int)config.limits.steerCurrentLimit.in(Amps));

        steerConfig.encoder.positionConversionFactor(1.0).velocityConversionFactor(1.0);

        steerConfig.closedLoop
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(
                        0.0,
                        config.physical.steerGearing.unapply(UnitConstants.INSTANCE.getHalfRotation().in(Rotations))
                )
                .pidf(
                        config.control.steerPidf.getP(),
                        config.control.steerPidf.getI(),
                        config.control.steerPidf.getD(),
                        config.control.steerPidf.getF()
                );

        this.steerController.clearFaults();
        this.steerController.configure(steerConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    private void configureAbsoluteEncoderInterface() {
        CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();

        absoluteEncoderConfig.MagnetSensor.withSensorDirection(config.state.steerWheelPositiveDirection.toSensorDirectionValue())
                                          .withMagnetOffset(config.state.absoluteEncoderOffset);

        this.absoluteEncoder.clearStickyFaults();
        this.absoluteEncoder.getConfigurator().apply(absoluteEncoderConfig);
    }

    // //// //
    // Data //
    // //// //

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.addDoubleProperty("Abs Azimuth (deg)", () -> getAbsoluteSteerWheelPosition().in(Degrees), (double m) -> {});
        sendableBuilder.addDoubleProperty("Rel Azimuth (deg)", () -> getSteerWheelAngularPosition().in(Degrees), (double m) -> {});
        sendableBuilder.addDoubleProperty("Tgt Azimuth (deg)", () -> targetState.angle.getDegrees(), (double m) -> {});

    }

    // //////////// //
    // Conversions //
    // //////////// //

    private Angle steerWheelAngularPositionToSteerShaftAngularPosition(Angle steerWheelAngle) {
        return config.physical.steerGearing.unapply(steerWheelAngle);
    }

    private AngularVelocity driveWheelLinearVelocityToDriveShaftAngularVelocity(LinearVelocity driveWheelVelocity) {
        return config.physical.driveGearing.unapply(config.physical.wheel.linearVelocityToAngularVelocity(driveWheelVelocity));
    }
}
