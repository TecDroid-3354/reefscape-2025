package net.tecdroid.subsystems.wrist

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.NeutralModeValue.Brake
import com.ctre.phoenix6.signals.NeutralModeValue.Coast
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import net.tecdroid.subsystems.util.motors.KrakenMotors
import net.tecdroid.util.amps
import net.tecdroid.util.degrees
import net.tecdroid.util.hertz
import net.tecdroid.util.volts
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

class WristIOPhoenix6(private val config: WristConfig): WristIO {
    private val motorController = KrakenMotors.createTalonWithFullConfig(
        config.motorControllerId, // Motor controller ID
        KrakenMotors.configureMotorOutputs(Brake, config.motorDirection.toInvertedValue()), // MotorOutputConfigs
        KrakenMotors.configureCurrentLimits(config.motorCurrentLimit, false, 0.0.amps),  // CurrentLimitConfigs
        KrakenMotors.configureSlot0(config.controlGains), config.reduction, // Slot0Configs, Subsystem's Reduction
        config.motionTargets, null, null // AngularMotionTargets
    )
    private val absoluteEncoder = ThroughBoreAbsoluteEncoder(
        port = config.absoluteEncoderPort,
        offset = config.absoluteEncoderOffset,
        inverted = config.absoluteEncoderIsInverted
    )

    private var motorPosition: StatusSignal<Angle> = motorController.position
    private var motorVoltage: StatusSignal<Voltage> = motorController.motorVoltage
    private var motorSupplyCurrent: StatusSignal<Current> = motorController.supplyCurrent
    private var motorTemperature: StatusSignal<Temperature> = motorController.deviceTemp

    private val voltageRequest = VoltageOut(0.0.volts) // Voltage request initialized here to optimize space.
    private val motionMagicRequest = MotionMagicVoltage(0.0.degrees)
        .withSlot(0) // MotionMagic request initialized here to optimize space.

    private var targetAngle: Angle = 0.0.degrees

    init {
        // Set the signals' update frequency to 50hz to match the robot's periodic cycle.
        BaseStatusSignal.setUpdateFrequencyForAll(50.0.hertz, motorPosition, motorVoltage, motorSupplyCurrent, motorTemperature)
        motorController.optimizeBusUtilization() // Refreshes every 100ms.
    }

    override fun setVoltage(voltage: Voltage) { motorController.setControl(voltageRequest.withOutput(voltage)) }
    override fun setAngle(angle: Angle) { motorController.setControl(motionMagicRequest.withPosition(angle)) }
    override fun setMotorPosition(angle: Angle) { motorController.setPosition(angle) }
    override fun setTargetAngle(angle: Angle) { targetAngle = angle }

    override fun getAbsoluteEncoderInstance(): ThroughBoreAbsoluteEncoder { return absoluteEncoder }

    override fun getMotorPosition(): Angle { return motorPosition.value }
    override fun getMotorVelocity(): AngularVelocity { return motorController.velocity.value }
    override fun getMotorPower(): Double { return motorController.get() }

    override fun updateInputs(inputs: WristIOInputs) {
        inputs.isThroughBoreConnected = absoluteEncoder.isConnected() // Encoder built-in check.
        inputs.throughBoreAbsolutePosition = absoluteEncoder.position

        inputs.isMotorConnected = BaseStatusSignal.refreshAll(
            motorPosition, motorVoltage, motorSupplyCurrent, motorTemperature
        ).isOK // If every signal refreshed successfully, the motor must be connected.

        inputs.motorPosition = motorPosition.value
        inputs.motorVoltage = motorVoltage.value
        inputs.motorSupplyCurrent = motorSupplyCurrent.value
        inputs.motorTemperature = motorTemperature.value
        inputs.wristTargetAngle = targetAngle
        inputs.wristAngle = config.reduction.apply(motorPosition.value)
    }

    override fun coast() { motorController.setNeutralMode(Coast) }

    override fun brake() { motorController.setNeutralMode(Brake) }
}