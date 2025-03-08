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
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import net.tecdroid.constants.UnitConstants.halfRotation
import net.tecdroid.kt.toRotation2d

/**
 * Represents a single module of a [SwerveDrive]
 *
 * @param config The configuration for this module
 */
class SwerveModule(private val config: SwerveModuleConfig) : Sendable {
    private val driveInterface = TalonFX(config.driveControllerId.id)
    private val steerController = SparkMax(config.steerControllerId.id, SparkLowLevel.MotorType.kBrushless)
    private val absoluteEncoder = CANcoder(config.absoluteEncoderId.id)

    private val steerEncoder = steerController.encoder
    private val steerClosedLoopController = steerController.closedLoopController

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
        setTargetAngle(targetState.angle.measure)
        setTargetVelocity(MetersPerSecond.of(targetState.speedMetersPerSecond))
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
        get() = config.driveGearRatio.apply(driveMotorShaftPosition)

    /**
     * The accumulated linear displacement of the module's wheel
     */
    val wheelLinearDisplacement: Distance
        get() = config.wheel.angularDisplacementToLinearDisplacement(wheelAngularDisplacement)

    /**
     * The angular velocity of the module's drive motor shaft
     */
    private val driveMotorShaftAngularVelocity: AngularVelocity
        get() = driveInterface.velocity.value

    /**
     * The angular velocity of the module's wheel
     */
    private val wheelAngularVelocity: AngularVelocity
        get() = config.driveGearRatio.apply(driveMotorShaftAngularVelocity)

    /**
     * The linear velocity of the module's wheel
     */
    val wheelLinearVelocity: LinearVelocity
        get() = config.wheel.angularVelocityToLinearVelocity(wheelAngularVelocity)

    /**
     * The max angular velocity of the drive motor's shaft
     */
    private val driveMotorShaftMaxAngularVelocity: AngularVelocity
        get() = config.driveMotorProperties.maxAngularVelocity

    /**
     * The max linear velocity of the drive wheel
     */
    val wheelMaxLinearVelocity: LinearVelocity
        get() = config.wheel.angularVelocityToLinearVelocity(
            config.driveGearRatio.apply(
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
        get() = config.steerGearRatio.unapply(absoluteWheelAzimuth)

    /**
     * The azimuth of the module's steer motor shaft
     */
    private val steerShaftAzimuth: Angle
        get() = Rotations.of(steerEncoder.position)

    /**
     * The azimuth of the module's wheel
     */
    val wheelAzimuth: Angle
        get() = config.steerGearRatio.apply(steerShaftAzimuth)

    /**
     * The state of the module as a [SwerveModuleState]
     */
    val state: SwerveModuleState
        get() = SwerveModuleState(wheelLinearVelocity, steerShaftAzimuth.toRotation2d())

    /**
     * The position of the module as a [SwerveModulePosition]
     */
    val position: SwerveModulePosition
        get() = SwerveModulePosition(wheelLinearDisplacement, steerShaftAzimuth.toRotation2d())

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
        return config.steerGearRatio.unapply(wheelAzimuth)
    }

    /**
     * Converts from a wheel linear velocity to a drive motor shaft angular velocity
     *
     * @param wheelVelocity The wheel velocity
     *
     * @return The drive motor shaft angular velocity
     */
    private fun wheelLinearVelocityToDriveMotorShaftAngularVelocity(wheelVelocity: LinearVelocity): AngularVelocity {
        return config.driveGearRatio.unapply(config.wheel.linearVelocityToAngularVelocity(wheelVelocity))
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
        state.optimize(wheelAzimuth.toRotation2d())
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

            CurrentLimits.withSupplyCurrentLimit(config.driveCurrentLimit)
                .withSupplyCurrentLimitEnable(true)

            Slot0.withKP(config.driveControlGains.p).withKI(config.driveControlGains.i)
                .withKD( config.driveControlGains.d).withKS(config.driveControlGains.s)
                .withKV( config.driveControlGains.v)
                .withKA( config.driveControlGains.a)

            MotorOutput.withNeutralMode(NeutralModeValue.Brake).withInverted(
                config.driveGearRatio.transformRotation(config.drivePositiveDirection)
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
                config.steerGearRatio.transformRotation(config.steerPositiveDirection)
                    .differs(
                        config.steerMotorProperties.positiveDirection
                    )
            ).smartCurrentLimit(config.steerCurrentLimit.`in`(Amps).toInt())

            encoder.positionConversionFactor(1.0).velocityConversionFactor(1.0)

            closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(
                0.0,
                config.steerGearRatio.unapply(halfRotation.`in`(Rotations))
            ).pidf(
                config.steerControlGains.p,
                config.steerControlGains.i,
                config.steerControlGains.d,
                config.steerControlGains.f
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
            MagnetSensor.withSensorDirection(config.steerPositiveDirection.toSensorDirectionValue())
                .withMagnetOffset(config.absoluteEncoderMagnetOffset)
        }

        absoluteEncoder.clearStickyFaults()
        absoluteEncoder.configurator.apply(absoluteEncoderConfig)
    }

}