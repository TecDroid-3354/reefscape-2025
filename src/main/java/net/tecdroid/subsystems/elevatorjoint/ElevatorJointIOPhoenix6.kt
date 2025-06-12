package net.tecdroid.subsystems.elevatorjoint

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

class ElevatorJointIOPhoenix6(private var config: ElevatorJointConfig): ElevatorJointIO {
    private val motorsConfig = KrakenMotors.createTalonFullConfig( // Stores the shared configuration of the motors.
        KrakenMotors.configureMotorOutputs(Brake, config.motorDirection.toInvertedValue()),
        KrakenMotors.configureCurrentLimits(config.motorCurrentLimit, false, 0.0.amps),
        KrakenMotors.configureSlot0(config.controlGains), config.reduction,
        config.motionTargets, null, null
    )
    private val leadMotorController = KrakenMotors.createTalonWithCustomConfig(
        config.leadMotorControllerId, motorsConfig
    )
    private val followerMotorController = KrakenMotors.createFollowerTalonWithCustomConfig(
        config.followerMotorControllerId, config.leadMotorControllerId, false, motorsConfig
    )

    private var leadMotorPosition: StatusSignal<Angle> = leadMotorController.position
    private var followerMotorPosition: StatusSignal<Angle> = followerMotorController.position
    private var leadMotorVoltage: StatusSignal<Voltage> = leadMotorController.motorVoltage
    private var followerMotorVoltage: StatusSignal<Voltage> = followerMotorController.motorVoltage
    private var leadMotorSupplyCurrent: StatusSignal<Current> = leadMotorController.supplyCurrent
    private var followerMotorSupplyCurrent: StatusSignal<Current> = followerMotorController.supplyCurrent
    private var leadMotorTemperature: StatusSignal<Temperature> = leadMotorController.deviceTemp
    private var followerMotorTemperature: StatusSignal<Temperature> = followerMotorController.deviceTemp

    private val absoluteEncoder =
        ThroughBoreAbsoluteEncoder(
            port = config.absoluteEncoderPort,
            offset = config.absoluteEncoderOffset,
            inverted = config.absoluteEncoderIsInverted
        )

    private var targetAngle = 0.0.degrees

    private val voltageRequest = VoltageOut(0.0.volts)
    private val motionMagicRequest = MotionMagicVoltage(0.0.degrees)

    init {
         BaseStatusSignal.setUpdateFrequencyForAll(50.0.hertz,
             leadMotorPosition, followerMotorPosition, leadMotorVoltage, followerMotorVoltage,
             leadMotorSupplyCurrent, followerMotorSupplyCurrent, leadMotorTemperature, followerMotorTemperature
         )
        leadMotorController.optimizeBusUtilization()
        followerMotorController.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: ElevatorJointIO.ElevatorJointIOInputs) {
        inputs.isThroughBoreConnected = absoluteEncoder.isConnected()
        inputs.throughBoreAbsolutePosition = absoluteEncoder.position
        inputs.isLeadMotorConnected = BaseStatusSignal.refreshAll(
            leadMotorPosition, leadMotorVoltage,
            leadMotorSupplyCurrent, leadMotorTemperature
        ).isOK
        inputs.isFollowerMotorConnected = BaseStatusSignal.refreshAll(
            followerMotorPosition, followerMotorVoltage,
            followerMotorSupplyCurrent, followerMotorTemperature
        ).isOK
        inputs.leadMotorPosition = leadMotorPosition.value
        inputs.followerMotorPosition = followerMotorPosition.value
        inputs.leadMotorVoltage = leadMotorVoltage.value
        inputs.followerMotorVoltage = followerMotorVoltage.value
        inputs.leadMotorSupplyCurrent = leadMotorSupplyCurrent.value
        inputs.followerMotorSupplyCurrent = followerMotorSupplyCurrent.value
        inputs.leadMotorTemperature = leadMotorTemperature.value
        inputs.followerMotorTemperature = followerMotorTemperature.value
        inputs.elevatorJointTargetAngle = targetAngle
        inputs.elevatorJointAngle = config.reduction.apply(leadMotorPosition.value)
    }

    override fun setVoltage(voltage: Voltage) { leadMotorController.setControl(voltageRequest.withOutput(voltage)) }

    override fun setAngle(angle: Angle) { leadMotorController.setControl(motionMagicRequest.withPosition(angle)) }

    override fun setMotorPosition(position: Angle) { leadMotorController.setPosition(position) }

    override fun setTargetAngle(angle: Angle) { targetAngle = angle }

    override fun getTargetAngle(): Angle { return targetAngle }

    override fun getAbsoluteEncoderInstance(): ThroughBoreAbsoluteEncoder { return absoluteEncoder }

    override fun getMotorPosition(): Angle { return leadMotorController.position.value }

    override fun getMotorVelocity(): AngularVelocity { return leadMotorController.velocity.value }

    override fun getMotorPower(): Double { return leadMotorController.get() }

    override fun coast() {
        leadMotorController.setNeutralMode(Coast)
        followerMotorController.setNeutralMode(Coast)
    }

    override fun brake() {
        leadMotorController.setNeutralMode(Brake)
        followerMotorController.setNeutralMode(Brake)
    }
}