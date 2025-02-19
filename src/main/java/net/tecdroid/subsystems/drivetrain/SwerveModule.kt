package net.tecdroid.subsystems.drivetrain

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.ControlType
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import net.tecdroid.constants.UnitConstants.halfRotation
import net.tecdroid.util.*
import net.tecdroid.util.geometry.Wheel

/**
 * Represents a single module of a [SwerveDrive]
 *
 * @param config The configuration for this module
 */
class SwerveModule(private val config: Config) : Sendable {
    private val driveInterface = TalonFX(config.identifiers.driveId.id)
    private val steerController = SparkMax(config.identifiers.steerId.id, SparkLowLevel.MotorType.kBrushless)
    private val steerEncoder = steerController.encoder
    private val steerClosedLoopController = steerController.closedLoopController
    private val absoluteEncoder = CANcoder(config.identifiers.absoluteEncoderId.id)

    init {
        this.configureDriveInterface()
        this.configureSteerInterface()
        this.configureAbsoluteEncoderInterface()
    }

    /**
     * Sets the module's target velocity
     *
     * @param velocity The velocity
     */
    private fun setTargetVelocity(velocity: LinearVelocity) {
        val driveShaftVelocity = wheelLinearVelocityToDriveMotorShaftAngularVelocity(velocity)
        driveInterface.setControl(VelocityVoltage(driveShaftVelocity).withSlot(0))
    }

    /**
     * Sets the module's target angle
     *
     * @param angle The angle
     */
    private fun setTargetAngle(angle: Angle) {
        val steerShaftAngle = wheelAzimuthToSteerMotorShaftAzimuth(angle)
        steerClosedLoopController.setReference(steerShaftAngle.`in`(Rotations), ControlType.kPosition)
    }

    /**
     * Sets the module's target state, optimizing it prior
     *
     * @param targetState The state
     */
    fun setTargetState(targetState: SwerveModuleState) {
        optimizeState(targetState)
        setTargetVelocity(MetersPerSecond.of(targetState.speedMetersPerSecond))
        setTargetAngle(targetState.angle.measure)
    }

    /**
     * Assigns the position of the absolute encoder to the steering encoder
     */
    fun matchSteeringEncoderToAbsoluteEncoder() {
        steerEncoder.setPosition(absoluteSteerShaftAzimuth.`in`(Rotations))
    }

    // ///////////////// //
    // Getters + Setters //
    // ///////////////// //
    /**
     * The accumulated angular displacement of the drive motor shaft
     */
    private val driveMotorShaftPosition: Angle
        get() = driveInterface.position.value

    /**
     * The accumulated angular displacement of the module's wheel
     */
    private val wheelAngularDisplacement: Angle
        get() = config.physical.driveGearing.apply(driveMotorShaftPosition)

    /**
     * The accumulated linear displacement of the module's wheel
     */
    private val wheelLinearDisplacement: Distance
        get() = config.physical.wheel.angularDisplacementToLinearDisplacement(wheelAngularDisplacement)

    /**
     * The angular velocity of the module's drive motor shaft
     */
    private val driveMotorShaftAngularVelocity: AngularVelocity
        get() = driveInterface.velocity.value

    /**
     * The angular velocity of the module's wheel
     */
    private val wheelAngularVelocity: AngularVelocity
        get() = config.physical.driveGearing.apply(driveMotorShaftAngularVelocity)

    /**
     * The linear velocity of the module's wheel
     */
    private val wheelLinearVelocity: LinearVelocity
        get() = config.physical.wheel.angularVelocityToLinearVelocity(wheelAngularVelocity)

    /**
     * The max angular velocity of the drive motor's shaft
     */
    private val driveMotorShaftMaxAngularVelocity: AngularVelocity
        get() = config.deviceProperties.driveMotorProperties.maxAngularVelocity

    /**
     * The max linear velocity of the drive wheel
     */
    val wheelMaxLinearVelocity: LinearVelocity
        get() = config.physical.wheel.angularVelocityToLinearVelocity(
            config.physical.driveGearing.apply(
                driveMotorShaftMaxAngularVelocity
            )
        )

    /**
     * The azimuth of the module's wheel (as indicated by absolute encoder)
     */
    private val absoluteWheelAzimuth: Angle
        get() = absoluteEncoder.position.value

    /**
     * The azimuth of the module's steer motor shaft (as indicated by absolute encoder)
     */
    private val absoluteSteerShaftAzimuth: Angle
        get() = config.physical.steerGearing.unapply(absoluteWheelAzimuth)

    /**
     * The azimuth of the module's steer motor shaft
     */
    private val steerShaftAzimuth: Angle
        get() = Rotations.of(steerEncoder.position)

    /**
     * The azimuth of the module's wheel
     */
    private val wheelAzimuth: Angle
        get() = config.physical.steerGearing.apply(steerShaftAzimuth)

    /**
     * The state of the module as a [SwerveModuleState]
     */
    val state: SwerveModuleState
        get() = SwerveModuleState(wheelLinearVelocity, Rotation2d(steerShaftAzimuth))

    /**
     * The position of the module as a [SwerveModulePosition]
     */
    val position: SwerveModulePosition
        get() = SwerveModulePosition(wheelLinearDisplacement, Rotation2d(steerShaftAzimuth))

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
    private fun wheelAzimuthToSteerMotorShaftAzimuth(wheelAzimuth: Angle): Angle {
        return config.physical.steerGearing.unapply(wheelAzimuth)
    }

    /**
     * Converts from a wheel linear velocity to a drive motor shaft angular velocity
     *
     * @param wheelVelocity The wheel velocity
     *
     * @return The drive motor shaft angular velocity
     */
    private fun wheelLinearVelocityToDriveMotorShaftAngularVelocity(wheelVelocity: LinearVelocity): AngularVelocity {
        return config.physical.driveGearing.unapply(config.physical.wheel.linearVelocityToAngularVelocity(wheelVelocity))
    }

    // //////// //
    // Sendable //
    // //////// //

    override fun initSendable(sendableBuilder: SendableBuilder) {
        sendableBuilder.addDoubleProperty(
            "Abs Azimuth (deg)",
            { absoluteWheelAzimuth.`in`(Degrees) },
            { })
        sendableBuilder.addDoubleProperty(
            "Rel Azimuth (deg)",
            { wheelAzimuth.`in`(Degrees) },
            { })
    }

    // //// //
    // Misc //
    // //// //

    /**
     * Optimizes a swerve module state
     *
     * @param state The state to optimize
     */
    private fun optimizeState(state: SwerveModuleState) {
        state.optimize(Rotation2d(wheelAzimuth))
    }

    // ///////////// //
    // Configuration //
    // ///////////// //

    /**
     * Configures the module's drive interface, that is, the controller that will take care of driving the wheel's
     * velocity
     */
    private fun configureDriveInterface() {
        val driveConfig = TalonFXConfiguration()

        with(driveConfig) {
            Audio.withBeepOnBoot(true).withBeepOnConfig(true).withAllowMusicDurDisable(true)

            CurrentLimits.withSupplyCurrentLimit(config.limits.driveCurrentLimit)
                .withSupplyCurrentLimitEnable(true)

            Slot0.withKP(config.control.drivePidf.p).withKI(config.control.drivePidf.i)
                .withKD(config.control.drivePidf.d).withKS(config.control.driveSvag.s)
                .withKV(config.control.driveSvag.v)
                .withKA(config.control.driveSvag.a)

            MotorOutput.withNeutralMode(NeutralModeValue.Brake).withInverted(
                config.physical.driveGearing.transformRotation(config.deviceConventions.driveWheelPositiveDirection)
                    .toInvertedValue()
            )
        }

        driveInterface.clearStickyFaults()
        driveInterface.configurator.apply(driveConfig)
    }

    /**
     * Configures the module's steer interface, that is, the controller that will take care of driving the wheel's
     * azimuth
     */
    private fun configureSteerInterface() {
        val steerConfig = SparkMaxConfig()

        with(steerConfig) {

            idleMode(IdleMode.kBrake).inverted(
                config.physical.steerGearing.transformRotation(config.deviceConventions.steerWheelPositiveDirection)
                    .differs(
                        config.deviceProperties.steerMotorProperties.positiveDirection
                    )
            ).smartCurrentLimit(config.limits.steerCurrentLimit.`in`(Amps).toInt())

            encoder.positionConversionFactor(1.0).velocityConversionFactor(1.0)

            closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(
                0.0,
                config.physical.steerGearing.unapply(halfRotation.`in`(Rotations))
            ).pidf(
                config.control.steerPidf.p,
                config.control.steerPidf.i,
                config.control.steerPidf.d,
                config.control.steerPidf.f
            )
        }


        steerController.clearFaults()
        steerController.configure(
            steerConfig, SparkBase.ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        )
    }

    /**
     * Configures the module's absolute encoder
     */
    private fun configureAbsoluteEncoderInterface() {
        val absoluteEncoderConfig = CANcoderConfiguration()

        with(absoluteEncoderConfig) {
            MagnetSensor.withSensorDirection(config.deviceConventions.steerWheelPositiveDirection.toSensorDirectionValue())
                .withMagnetOffset(config.specifics.absoluteEncoderOffset)
        }

        absoluteEncoder.clearStickyFaults()
        absoluteEncoder.configurator.apply(absoluteEncoderConfig)
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
    @JvmRecord
    data class DeviceIdentifiers(val driveId: NumericId, val steerId: NumericId, val absoluteEncoderId: NumericId)

    @JvmRecord
    data class DeviceProperties(val driveMotorProperties: MotorProperties, val steerMotorProperties: MotorProperties)

    /**
     * Stores limits that regulate the module's motion
     *
     * @param driveCurrentLimit The current limit for the driving motor
     * @param steerCurrentLimit The current limit for the steering motor
     */
    @JvmRecord
    data class DeviceLimits(val driveCurrentLimit: Current, val steerCurrentLimit: Current)

    /**
     * Stores the considerations that must be taken into account due to the module's physical state
     *
     * @param driveWheelPositiveDirection The direction in which the drive wheel must turn when it receives a
     * positive input
     * @param steerWheelPositiveDirection The direction in which the steer wheel must turn when it receives a
     * positive input
     */
    @JvmRecord
    data class DeviceConventions(
        val driveWheelPositiveDirection: RotationalDirection,
        val steerWheelPositiveDirection: RotationalDirection
    )

    /**
     * Stores the configuration parameters that are unique to each module
     *
     * @param absoluteEncoderOffset The magnet offset of the absolute encoder
     */
    @JvmRecord
    data class ModuleSpecifics(val absoluteEncoderOffset: Angle)

    /**
     * Stores characteristics relating to the module's physical description
     *
     * @param driveGearing The gearing between the drive shaft and the wheel
     * @param steerGearing The gearing between the steer shaft and the wheel
     * @param wheel        The wheel's physical description
     */
    @JvmRecord
    data class PhysicalDescription(val driveGearing: GearRatio, val steerGearing: GearRatio, val wheel: Wheel)

    /**
     * Stores the control constants that will be applied to the module's motion
     *
     * @param drivePidf The PIDF feedback coefficients used to control the driving motion
     * @param driveSvag The SVAG feedforward gains used to control the driving motion
     * @param steerPidf The PIDF feedback coefficients used to control the steering motion
     * @param steerSvag The SVAG feedforward gains used to control the steering motion
     */
    @JvmRecord
    data class ControlConstants(
        val drivePidf: PidfCoefficients, val driveSvag: SvagGains, val steerPidf: PidfCoefficients,
        val steerSvag: SvagGains
    )

    /**
     * Stores the module's configuration parameters
     *
     * @param identifiers The identifier config
     * @param physical    The physical config
     * @param control     The control config
     * @param limits      The limt config
     */
    @JvmRecord
    data class Config(
        val identifiers: DeviceIdentifiers, val deviceProperties: DeviceProperties,
        val deviceConventions: DeviceConventions, val limits: DeviceLimits, val physical: PhysicalDescription,
        val control: ControlConstants, val specifics: ModuleSpecifics
    )
}
