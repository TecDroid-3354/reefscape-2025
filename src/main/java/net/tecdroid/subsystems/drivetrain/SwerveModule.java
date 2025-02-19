package net.tecdroid.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    private final Config                    config;
    private final TalonFX                   driveInterface;
    private final SparkMax                  steerController;
    private final RelativeEncoder           steerEncoder;
    private final SparkClosedLoopController steerClosedLoopController;
    private final CANcoder                  absoluteEncoder;

    /**
     * Represents a single module in a swerve drive
     *
     * @param config The configuration for this module
     */
    public SwerveModule(Config config) {
        this.config = config;

        this.driveInterface = new TalonFX(config.identifiers.driveId.getId());
        this.configureDriveInterface();

        this.steerController = new SparkMax(config.identifiers.steerId.getId(), MotorType.kBrushless);
        this.steerEncoder = steerController.getEncoder();
        this.steerClosedLoopController = steerController.getClosedLoopController();
        this.configureSteerInterface();

        this.absoluteEncoder = new CANcoder(config.identifiers.absoluteEncoderId.getId());
        this.configureAbsoluteEncoderInterface();
    }

    /**
     * Sets the module's target velocity
     *
     * @param velocity The velocity
     */
    public void setTargetVelocity(LinearVelocity velocity) {
        final AngularVelocity driveShaftVelocity = wheelLinearVelocityToDriveMotorShaftAngularVelocity(velocity);
        driveInterface.setControl(new VelocityVoltage(driveShaftVelocity).withSlot(0));
    }

    /**
     * Sets the module's target angle
     *
     * @param angle The angle
     */
    public void setTargetAngle(Angle angle) {
        final Angle steerShaftAngle = wheelAzimuthToSteerMotorShaftAzimuth(angle);
        steerClosedLoopController.setReference(steerShaftAngle.in(Rotations), ControlType.kPosition);
    }

    public void test() {
        steerController.set(0.05);
    }

    /**
     * Sets the module's target state, optimizing it prior
     *
     * @param targetState The state
     */
    public void setTargetState(SwerveModuleState targetState) {
        optimizeState(targetState);
        setTargetVelocity(MetersPerSecond.of(targetState.speedMetersPerSecond));
        setTargetAngle(targetState.angle.getMeasure());
    }

    /**
     * Assigns the position of the absolute encoder to the steering encoder
     */
    public void matchSteeringEncoderToAbsoluteEncoder() {
        steerEncoder.setPosition(getAbsoluteSteerShaftAzimuth().in(Rotations));
    }

    // ///////////////// //
    // Getters + Setters //
    // ///////////////// //

    /**
     * @return The accumulated angular displacement of the drive motor shaft
     */
    public Angle getDriveMotorShaftPosition() {
        return driveInterface.getPosition().getValue();
    }

    /**
     * @return The accumulated angular displacement of the module's wheel
     */
    public Angle getWheelAngularDisplacement() {
        return config.physical.driveGearing.apply(getDriveMotorShaftPosition());
    }

    /**
     * @return The accumulated linear displacement of the module's wheel
     */
    public Distance getWheelLinearDisplacement() {
        return config.physical.wheel.angularDisplacementToLinearDisplacement(getWheelAngularDisplacement());
    }

    /**
     * @return The angular velocity of the module's drive motor shaft
     */
    public AngularVelocity getDriveMotorShaftAngularVelocity() {
        return driveInterface.getVelocity().getValue();
    }

    /**
     * @return The angular velocity of the module's wheel
     */
    public AngularVelocity getWheelAngularVelocity() {
        return config.physical.driveGearing.apply(getDriveMotorShaftAngularVelocity());
    }

    /**
     * @return The linear velocity of the module's wheel
     */
    public LinearVelocity getWheelLinearVelocity() {
        return config.physical.wheel.angularVelocityToLinearVelocity(getWheelAngularVelocity());
    }

    public AngularVelocity getDriveMotorShaftMaxAngularVelocity() {
        return config.deviceProperties.driveMotorProperties.getMaxAngularVelocity();
    }

    public LinearVelocity getWheelMaxLinearVelocity() {
        return config.physical.wheel.angularVelocityToLinearVelocity(config.physical.driveGearing.apply(getDriveMotorShaftMaxAngularVelocity()));
    }

    /**
     * @return The azimuth of the module's wheel (as indicated by absolute encoder)
     */
    public Angle getAbsoluteWheelAzimuth() {
        return absoluteEncoder.getPosition().getValue();
    }

    /**
     * @return The azimuth of the module's steer motor shaft (as indicated by absolute encoder)
     */
    public Angle getAbsoluteSteerShaftAzimuth() {
        return config.physical.steerGearing.unapply(getAbsoluteWheelAzimuth());
    }

    /**
     * @return The azimuth of the module's steer motor shaft
     */
    public Angle getSteerShaftAzimuth() {
        return Rotations.of(steerEncoder.getPosition());
    }

    /**
     * @return The azimuth of the module's wheel
     */
    public Angle getWheelAzimuth() {
        return config.physical.steerGearing.apply(getSteerShaftAzimuth());
    }

    /**
     * @return The state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelLinearVelocity(), new Rotation2d(getSteerShaftAzimuth()));
    }

    /**
     * @return The position of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getWheelLinearDisplacement(), new Rotation2d(getSteerShaftAzimuth()));
    }

    // /////////// //
    // Conversions //
    // /////////// //

    /**
     * Converts from a wheel azimuth to a steer motor shaft azimuth
     *
     * @param wheelAzimuth The wheel azimuth
     *
     * @return The steer motor shaft azimuth
     */
    private Angle wheelAzimuthToSteerMotorShaftAzimuth(Angle wheelAzimuth) {
        return config.physical.steerGearing.unapply(wheelAzimuth);
    }

    /**
     * Converts from a wheel linear velocity to a drive motor shaft angular velocity
     *
     * @param wheelVelocity The wheel velocity
     *
     * @return The drive motor shaft angular velocity
     */
    private AngularVelocity wheelLinearVelocityToDriveMotorShaftAngularVelocity(LinearVelocity wheelVelocity) {
        return config.physical.driveGearing.unapply(config.physical.wheel.linearVelocityToAngularVelocity(wheelVelocity));
    }

    // //////// //
    // Sendable //
    // //////// //

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.addDoubleProperty("Abs Azimuth (deg)", () -> getAbsoluteWheelAzimuth().in(Degrees),
                                          (double m) -> {
        });
        sendableBuilder.addDoubleProperty("Rel Azimuth (deg)", () -> getWheelAzimuth().in(Degrees), (double m) -> {
        });
    }

    // //// //
    // Misc //
    // //// //

    /**
     * Optimizes a swerve module state
     *
     * @param state The state to optimize
     */
    private void optimizeState(SwerveModuleState state) {
        state.optimize(new Rotation2d(getWheelAzimuth()));
    }

    // ///////////// //
    // Configuration //
    // ///////////// //

    /**
     * Configures the module's drive interface, that is, the controller that will take care of driving the wheel's
     * velocity
     */
    private void configureDriveInterface() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.Audio.withBeepOnBoot(true).withBeepOnConfig(true).withAllowMusicDurDisable(true);

        driveConfig.CurrentLimits.withSupplyCurrentLimit(config.limits.driveCurrentLimit).withSupplyCurrentLimitEnable(true);

        driveConfig.Slot0.withKP(config.control.drivePidf.getP()).withKI(config.control.drivePidf.getI()).withKD(config.control.drivePidf.getD()).withKS(config.control.driveSvag.getS()).withKV(config.control.driveSvag.getV()).withKA(config.control.driveSvag.getA());

        driveConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake).withInverted(config.physical.driveGearing.transformRotation(config.deviceConventions.driveWheelPositiveDirection).toInvertedValue());

        driveInterface.clearStickyFaults();
        driveInterface.getConfigurator().apply(driveConfig);
    }

    /**
     * Configures the module's steer interface, that is, the controller that will take care of driving the wheel's
     * azimuth
     */
    private void configureSteerInterface() {
        SparkMaxConfig steerConfig = new SparkMaxConfig();

        steerConfig.idleMode(IdleMode.kBrake).inverted(config.physical.steerGearing.transformRotation(config.deviceConventions.steerWheelPositiveDirection).differs(Motors.INSTANCE.getNeo().getPositiveDirection())).smartCurrentLimit((int) config.limits.steerCurrentLimit.in(Amps));

        steerConfig.encoder.positionConversionFactor(1.0).velocityConversionFactor(1.0);

        steerConfig.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(0.0,
                                                                                        config.physical.steerGearing.unapply(UnitConstants.INSTANCE.getHalfRotation().in(Rotations))).pidf(config.control.steerPidf.getP(), config.control.steerPidf.getI(), config.control.steerPidf.getD(), config.control.steerPidf.getF());

        steerController.clearFaults();
        steerController.configure(steerConfig, SparkBase.ResetMode.kResetSafeParameters,
                                  PersistMode.kNoPersistParameters);
    }

    /**
     * Configures the module's absolute encoder
     */
    private void configureAbsoluteEncoderInterface() {
        CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();

        absoluteEncoderConfig.MagnetSensor.withSensorDirection(config.deviceConventions.steerWheelPositiveDirection.toSensorDirectionValue()).withMagnetOffset(config.specifics.absoluteEncoderOffset);

        this.absoluteEncoder.clearStickyFaults();
        this.absoluteEncoder.getConfigurator().apply(absoluteEncoderConfig);
    }

    // ///////////////////// //
    // Configuration Classes //
    // ///////////////////// //

    /**
     * Stores the device IDs of the module's electronic components
     *
     * @param driveId           The ID of the drive motor controller
     * @param steerId           The ID of the steer motor controller
     * @param absoluteEncoderId The ID of the absolute encoder
     */
    public record DeviceIdentifiers(NumericId driveId, NumericId steerId, NumericId absoluteEncoderId) {
    }

    public record DeviceProperties(MotorProperties driveMotorProperties, MotorProperties steerMotorProperties) {
    }

    /**
     * Stores limits that regulate the module's motion
     *
     * @param driveCurrentLimit The current limit for the driving motor
     * @param steerCurrentLimit The current limit for the steering motor
     */
    public record DeviceLimits(Current driveCurrentLimit, Current steerCurrentLimit) {
    }

    /**
     * Stores the considerations that must be taken into account due to the module's physical state
     *
     * @param driveWheelPositiveDirection The direction in which the drive wheel must turn when it receives a
     *                                    positive input
     * @param steerWheelPositiveDirection The direction in which the steer wheel must turn when it receives a
     *                                    positive input
     */
    public record DeviceConventions(RotationalDirection driveWheelPositiveDirection,
                                    RotationalDirection steerWheelPositiveDirection) {
    }

    /**
     * Stores the configuration parameters that are unique to each module
     *
     * @param absoluteEncoderOffset The magnet offset of the absolute encoder
     */
    public record ModuleSpecifics(Angle absoluteEncoderOffset) {
    }

    /**
     * Stores characteristics relating to the module's physical description
     *
     * @param driveGearing The gearing between the drive shaft and the wheel
     * @param steerGearing The gearing between the steer shaft and the wheel
     * @param wheel        The wheel's physical description
     */
    public record PhysicalDescription(GearRatio driveGearing, GearRatio steerGearing, Wheel wheel) {
    }

    /**
     * Stores the control constants that will be applied to the module's motion
     *
     * @param drivePidf The PIDF feedback coefficients used to control the driving motion
     * @param driveSvag The SVAG feedforward gains used to control the driving motion
     * @param steerPidf The PIDF feedback coefficients used to control the steering motion
     * @param steerSvag The SVAG feedforward gains used to control the steering motion
     */
    public record ControlConstants(PidfCoefficients drivePidf, SvagGains driveSvag, PidfCoefficients steerPidf,
                                   SvagGains steerSvag) {
    }

    /**
     * Stores the module's configuration parameters
     *
     * @param identifiers The identifier config
     * @param physical    The physical config
     * @param control     The control config
     * @param limits      The limt config
     */
    public record Config(DeviceIdentifiers identifiers, DeviceProperties deviceProperties,
                         DeviceConventions deviceConventions, DeviceLimits limits, PhysicalDescription physical,
                         ControlConstants control, ModuleSpecifics specifics) {
    }
}
