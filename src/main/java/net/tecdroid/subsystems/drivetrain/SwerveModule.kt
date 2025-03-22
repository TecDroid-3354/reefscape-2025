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
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import net.tecdroid.constants.UnitConstants.halfRotation
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.toRotation2d

/**
 * Represents a single module of a [SwerveDrive]
 *
 * @param config The configuration for this module
 */
class SwerveModule(private val config: SwerveModuleConfig) : Sendable {
    private val driveInterface = TalonFX(config.driveControllerId.id)
    private val steerController = SparkMax(config.steerControllerId.id, SparkLowLevel.MotorType.kBrushless)
    private val absoluteEncoder = CANcoder(config.absoluteEncoderId.id)

    private var tst = SwerveModuleState()

    private val steerEncoder = steerController.encoder
    private val steerClosedLoopController = steerController.closedLoopController

    init {
        this.configureDriveInterface()
        this.configureSteerInterface()
        this.configureAbsoluteEncoderInterface()
    }

    fun setPower(power: Double) {
        driveInterface.set(power)
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
     fun setTargetAngle(angle: Angle) {
        val steerShaftAngle = wheelAzimuthToSteerMotorShaftAzimuth(angle)
        steerClosedLoopController.setReference(steerShaftAngle.`in`(Rotations), ControlType.kPosition)
    }

    /**
     * Sets the module's target state, optimizing it prior
     *
     * @param targetState The state
     */
    fun setTargetState(targetState: SwerveModuleState) {
        tst = targetState
        optimizeState(targetState)
        setTargetAngle(targetState.angle.measure)
        setTargetVelocity(MetersPerSecond.of(targetState.speedMetersPerSecond))
    }

    fun matchRelativeEncodersToAbsoluteEncoders() {
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
        get() = config.circle.angularDisplacementToLinearDisplacement(wheelAngularDisplacement)

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
        get() = config.circle.angularVelocityToLinearVelocity(wheelAngularVelocity)

    /**
     * The max angular velocity of the drive motor's shaft
     */
    private val driveMotorShaftMaxAngularVelocity: AngularVelocity
        get() = config.driveMotorProperties.maxAngularVelocity

    /**
     * The max linear velocity of the drive wheel
     */
    val wheelMaxLinearVelocity: LinearVelocity
        get() = config.circle.angularVelocityToLinearVelocity(
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
        return config.driveGearRatio.unapply(config.circle.linearVelocityToAngularVelocity(wheelVelocity))
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
        sendableBuilder.addDoubleProperty(
            "Target Azimuth (deg)",
            { tst.angle.degrees },
            { })

        sendableBuilder.addDoubleProperty(
            "Velocity (m/s)",
            { wheelLinearVelocity.`in`(MetersPerSecond) },
            { })
        sendableBuilder.addIntegerProperty(
            "Integer",
            { config.driveControllerId.id.toLong() },
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
                config.drivePositiveDirection
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
                config.steerPositiveDirection.opposite() == config.steerMotorProperties.positiveDirection
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
            MagnetSensor.withSensorDirection(config.steerPositiveDirection.opposite().toSensorDirectionValue())
                .withMagnetOffset(config.absoluteEncoderMagnetOffset)
        }

        absoluteEncoder.clearStickyFaults()
        absoluteEncoder.configurator.apply(absoluteEncoderConfig)
    }

    fun align() {
        setTargetAngle(0.0.degrees)
    }

    // Creamos una propiedad que devuelve el estado del módulo
    val state: SwerveModuleState
        get() = SwerveModuleState(wheelLinearVelocity, wheelAzimuth.toRotation2d())


}