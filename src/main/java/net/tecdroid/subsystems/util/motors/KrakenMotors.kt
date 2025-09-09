package net.tecdroid.subsystems.util.motors

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Current
import net.tecdroid.mechanical.Reduction
import net.tecdroid.util.*

/**
 * Class meant to provide an easier, more concise way to configure [TalonFX] motor controllers within the project.
 * Please be aware of the default config written in this file, as that is the configuration that would be applied
 * for any missing user-provided configuration when creating a [TalonFX].
 */
object KrakenMotors {
    private val defaultConfig = TalonFXConfiguration()

    /**
     * Called after the primary constructor. Used to initialize the default config for all [TalonFX] motor controllers.
     */
    init {
        with(defaultConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)
            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0.amps)
                .withStatorCurrentLimitEnable(false)
                .withStatorCurrentLimit(120.0.amps)
        }
    }

    /**
     * Creates a [TalonFX] with project default, not Talon's, configuration, this means:
     * [NeutralModeValue] = Brake.
     * [InvertedValue] = CounterClockwise Positive.
     * [CurrentLimitsConfigs] = Supply current limit enabled with 40 amps, stator current limit disabled.
     * @param id The ID for the motor controller.
     * @return A [TalonFX] motor controller with the configuration described above. Sticky faults are cleared.
     */
    fun createDefaultTalon(id: NumericId): TalonFX {
        val talon = TalonFX(id.id)
        talon.clearStickyFaults()
        talon.configurator.apply(defaultConfig)
        return talon
    }

    /**
     * Creates a [TalonFX] with custom configuration. Note that you can type null for any configuration, causing it to
     * stay with the project's default config in said configuration, if applicable.
     * If you happen to include any type of MotionMagic control without its requirements, as described below, said
     * configuration would not be applied.
     * @param id The ID for the motor controller.
     * @param motorOutputs The desired [MotorOutputConfigs]. Note that you can use [configureMotorOutputs].
     * @param currentLimits The desired [CurrentLimitsConfigs]. Note that you can use [configureCurrentLimits].
     * @param slot0 The desired [Slot0Configs]. Note that you can use [configureSlot0].
     * @param reduction The desired [Reduction]. Note that this field is required if using any form of MotionMagic.
     * @param angularMotionTargets The desired [AngularMotionTargets]. Note that you can use [configureAngularMotionMagic].
     * @param linearMotionTargets The desired [LinearMotionTargets]. Note that you can use [configureLinearMotionMagic].
     * @param sprocket The desired [Sprocket]. Note that this field is required if using [linearMotionTargets].
     * @return A [TalonFX] motor controller with the desired configurations. Sticky faults are cleared.
     */
    fun createTalonWithFullConfig(id: NumericId, motorOutputs: MotorOutputConfigs?, currentLimits: CurrentLimitsConfigs?,
                                  slot0: Slot0Configs?, reduction: Reduction?, angularMotionTargets: AngularMotionTargets?,
                                  linearMotionTargets: LinearMotionTargets?, sprocket: Sprocket?)
    : TalonFX {
        val talon = TalonFX(id.id)
        val customConfig = defaultConfig

        if (motorOutputs != null) {
            customConfig.MotorOutput = motorOutputs
        }
        if (currentLimits != null) {
            customConfig.CurrentLimits = currentLimits
        }
        if (slot0 != null) {
            customConfig.Slot0 = slot0
        }
        if (angularMotionTargets != null && reduction != null) {
            customConfig.MotionMagic = configureAngularMotionMagic(customConfig, angularMotionTargets, reduction).MotionMagic
        }
        if (linearMotionTargets != null && reduction != null && sprocket != null) {
            customConfig.MotionMagic = configureLinearMotionMagic(customConfig, linearMotionTargets, reduction, sprocket).MotionMagic
        }

        talon.clearStickyFaults()
        talon.configurator.apply(customConfig)
        return talon
    }

    /**
     * Creates a [TalonFX] with all project default configs, with the exemption of Motion Magic and [ControlGains],
     * that are user-provided. [controlValues] are optional, if not desired type null.
     * Note that you must provide either [angularMotionTargets] or [linearMotionTargets], with its requirements.
     * If you happen to provide both, [linearMotionTargets] prevail. If you provide [linearMotionTargets] without
     * providing [reduction] AND [sprocket], the configuration would not be applied.
     * @param id The ID for the motor controller.
     * @param angularMotionTargets The desired [AngularMotionTargets]. If not applicable, type null.
     * @param linearMotionTargets The desired [LinearMotionTargets]. If not applicable, type null.
     * @param reduction The [Reduction] of the subsystem, only required if using [linearMotionTargets], otherwise type null.
     * @param sprocket The [Sprocket] of the subsystem, only required if using [linearMotionTargets], otherwise type null.
     * @return A [TalonFX] motor controller with project default configs plus the desired MotionMagic and
     *          control values (PIDF, SVAG). Sticky faults are cleared.
     */
    fun createTalonWithControlValues(
        id: NumericId, controlValues: ControlGains?, reduction: Reduction?, angularMotionTargets: AngularMotionTargets?,
        linearMotionTargets: LinearMotionTargets?, sprocket: Sprocket?): TalonFX {
        val talon = TalonFX(id.id)
        var customConfig = defaultConfig

        if (controlValues != null) {
            customConfig.Slot0 = configureSlot0(controlValues)
        }

        if (angularMotionTargets != null && reduction != null) {
            customConfig = configureAngularMotionMagic(customConfig, angularMotionTargets, reduction)
        }

        if (linearMotionTargets != null && reduction != null && sprocket != null) {
            customConfig = configureLinearMotionMagic(customConfig, linearMotionTargets, reduction, sprocket)
        }

        talon.clearStickyFaults()
        talon.configurator.apply(customConfig)
        return talon
    }

    /**
     * Intended to use alongside [createTalonFullConfig] in subsystems with a leading and a following motor. This allows
     * the user to first instantiate the shared configuration of the motors and then create the leading motor with said
     * configuration.
     * @param id The ID for the motor controller. (Most likely the lead motor of the subsystem).
     * @param motorConfig The [TalonFXConfiguration] to apply to the motor.
     * @return A [TalonFX] motor controller with the desired configuration. Sticky faults are cleared.
     */
    fun createTalonWithCustomConfig(id: NumericId, motorConfig: TalonFXConfiguration): TalonFX {
        val talon = TalonFX(id.id)

        talon.clearStickyFaults()
        talon.configurator.apply(motorConfig)

        return talon
    }

    /**
     * Intended to use alongside [createTalonFullConfig] in subsystems with a leading and a following motor. This allows
     * the user to first instantiate the shared configuration of the motors and then create the follower motor with said
     * configuration.
     * @param followerId The ID of the following motor controller.
     * @param masterId The ID of the leading motor controller.
     * @param opposeMaster Whether the follower motor should oppose its master's direction.
     * @param motorConfig The [TalonFXConfiguration] to apply to the follower motor.
     * @return A [TalonFX] motor controller with the desired configuration and with a [Follower] request to follow
     * the leading motor, specified by the [masterId]. Sticky faults are cleared.
     *
     * @see [createTalonWithCustomConfig]
     */
    fun createFollowerTalonWithCustomConfig(followerId: NumericId, masterId: NumericId,
                                            opposeMaster: Boolean, motorConfig: TalonFXConfiguration): TalonFX {
        val followerTalon = TalonFX(followerId.id)

        followerTalon.clearStickyFaults()
        followerTalon.configurator.apply(motorConfig)
        followerTalon.setControl(Follower(masterId.id, opposeMaster))
        return followerTalon
    }

    /**
     * Creates a custom [TalonFXConfiguration]. Note that you can type null for any configuration, causing it to
     * stay with the project's default config in said configuration, if applicable.
     * If you happen to include any type of MotionMagic control without its requirements, as described below, said
     * configuration would not be applied.
     * @param motorOutputs The desired [MotorOutputConfigs]. Note that you can use [configureMotorOutputs].
     * @param currentLimits The desired [CurrentLimitsConfigs]. Note that you can use [configureCurrentLimits].
     * @param slot0 The desired [Slot0Configs]. Note that you can use [configureSlot0].
     * @param reduction The desired [Reduction]. Note that this field is required if using any form of MotionMagic.
     * @param angularMotionTargets The desired [AngularMotionTargets]. Note that you can use [configureAngularMotionMagic].
     * @param linearMotionTargets The desired [LinearMotionTargets]. Note that you can use [configureLinearMotionMagic].
     * @param sprocket The desired [Sprocket]. Note that this field is required if using [linearMotionTargets].
     * @return A [TalonFXConfiguration] with the desired configurations.
     */
    fun createTalonFullConfig(motorOutputs: MotorOutputConfigs?, currentLimits: CurrentLimitsConfigs?,
                                  slot0: Slot0Configs?, reduction: Reduction?, angularMotionTargets: AngularMotionTargets?,
                                  linearMotionTargets: LinearMotionTargets?, sprocket: Sprocket?)
            : TalonFXConfiguration {
        val talonConfig = defaultConfig

        if (motorOutputs != null) {
            talonConfig.MotorOutput = motorOutputs
        }
        if (currentLimits != null) {
            talonConfig.CurrentLimits = currentLimits
        }
        if (slot0 != null) {
            talonConfig.Slot0 = slot0
        }
        if (angularMotionTargets != null && reduction != null) {
            talonConfig.MotionMagic = configureAngularMotionMagic(talonConfig, angularMotionTargets, reduction).MotionMagic
        }
        if (linearMotionTargets != null && reduction != null && sprocket != null) {
            talonConfig.MotionMagic = configureLinearMotionMagic(talonConfig, linearMotionTargets, reduction, sprocket).MotionMagic
        }

        return talonConfig
    }

    /**
     * Takes the desired motor outputs of a [TalonFXConfiguration] and returns a [MotorOutputConfigs] with said
     * configurations.
     * @param neutralModeValue The desired [NeutralModeValue], either Brake or Coast.
     * @param invertedValue The desired [InvertedValue], either CounterClockwise_Positive or Clockwise_Positive.
     * @return A [MotorOutputConfigs] with the desired [NeutralModeValue] and [InvertedValue].
     */
    fun configureMotorOutputs(neutralModeValue: NeutralModeValue, invertedValue: InvertedValue): MotorOutputConfigs {
        return MotorOutputConfigs()
            .withNeutralMode(neutralModeValue)
            .withInverted(invertedValue)
    }

    /**
     * Takes the desired current limits and returns a [CurrentLimitsConfigs] with said configurations. If stator limit
     * is not desired, type null.
     * @param supplyCurrentLimit The desired supply current limit.
     * @param statorCurrentLimit The desired stator current limit. Optional.
     * @return A [CurrentLimitsConfigs] with the desired supply and stator current limits, if applicable.
     */
    fun configureCurrentLimits(supplyCurrentLimit: Current,
                               statorCurrentLimitEnabled: Boolean, statorCurrentLimit: Current): CurrentLimitsConfigs {
        return CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(supplyCurrentLimit)
            .withStatorCurrentLimitEnable(statorCurrentLimitEnabled)
            .withStatorCurrentLimit(statorCurrentLimit)
    }

    /**
     * Takes the desired [ControlGains] and returns a [Slot0Configs] with said configurations.
     * @param controlGains The desired [ControlGains].
     * @return A [Slot0Configs] with the applied [ControlGains].
     */
    fun configureSlot0(controlGains: ControlGains): Slot0Configs {
        return Slot0Configs()
            .withKP(controlGains.p)
            .withKI(controlGains.i)
            .withKD(controlGains.d)
            .withKS(controlGains.s)
            .withKV(controlGains.v)
            .withKA(controlGains.a)
            .withKG(controlGains.g)
    }

    /**
     * Takes a [TalonFXConfiguration] and configures its MotionMagic's cruise velocity, acceleration and jerk.
     * @param talonConfig [TalonFXConfiguration] to apply the MotionMagic configuration to.
     * @param angularMotionTargets [AngularMotionTargets] containing the desired configuration.
     * @param reduction The subsystem's [Reduction].
     * @return Your [talonConfig] with MotionMagic configuration applied.
     */
    private fun configureAngularMotionMagic(talonConfig: TalonFXConfiguration,
                                            angularMotionTargets: AngularMotionTargets,
                                            reduction: Reduction): TalonFXConfiguration {
        with(talonConfig) {
            MotionMagic
                .withMotionMagicCruiseVelocity(reduction.unapply(angularMotionTargets.cruiseVelocity))
                .withMotionMagicAcceleration(reduction.unapply(angularMotionTargets.acceleration))
                .withMotionMagicJerk(reduction.unapply(angularMotionTargets.jerk))
        }

        return talonConfig
    }

    /**
     * Takes a [TalonFXConfiguration] and configures its MotionMagic's cruise velocity, acceleration and jerk.
     * This method takes [LinearMotionTargets], thus the subsystem's [Reduction] and [Sprocket] are necessary to
     * convert from linear to angular targets.
     * @param talonConfig [TalonFXConfiguration] to apply the MotionMagic configuration to.
     * @param linearMotionTargets [LinearMotionTargets] containing the desired configuration.
     * @param reduction [Reduction] of the subsystem.
     * @param sprocket [Sprocket] attached to the linear subsystem. Used to convert from linear to angular.
     * @return Your [talonConfig] with MotionMagic configurations applied.
     */
    private fun configureLinearMotionMagic(
        talonConfig: TalonFXConfiguration, linearMotionTargets: LinearMotionTargets,
        reduction: Reduction, sprocket: Sprocket): TalonFXConfiguration {
        with(talonConfig) {
            MotionMagic
                .withMotionMagicCruiseVelocity(reduction.unapply(linearMotionTargets.angularVelocity(sprocket)))
                .withMotionMagicAcceleration(reduction.unapply(linearMotionTargets.angularAcceleration(sprocket)))
                .withMotionMagicJerk(reduction.unapply(linearMotionTargets.angularJerk(sprocket)))
        }
        return talonConfig
    }
}
