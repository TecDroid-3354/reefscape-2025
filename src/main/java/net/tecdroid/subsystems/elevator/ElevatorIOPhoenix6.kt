package net.tecdroid.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.NeutralModeValue.Brake
import com.ctre.phoenix6.signals.NeutralModeValue.Coast
import edu.wpi.first.units.measure.*
import net.tecdroid.subsystems.util.motors.KrakenMotors
import net.tecdroid.util.amps
import net.tecdroid.util.degrees
import net.tecdroid.util.hertz
import net.tecdroid.util.volts

class ElevatorIOPhoenix6(private val config: ElevatorConfig): ElevatorIO {
    private val motorsConfig = KrakenMotors.createTalonFullConfig( // Stores the shared configuration of the motors.
        KrakenMotors.configureMotorOutputs(Brake, config.motorDirection.toInvertedValue()),
        KrakenMotors.configureCurrentLimits(config.motorCurrentLimit, false, 0.0.amps),
        KrakenMotors.configureSlot0(config.controlGains), config.reduction,
        null, config.motionTargets, config.sprocket
    )
    private val leadMotorController = KrakenMotors.createTalonWithCustomConfig(
        config.leadMotorControllerId, motorsConfig
    )
    private val followerMotorController = KrakenMotors.createFollowerTalonWithCustomConfig(
        config.followerMotorId, config.leadMotorControllerId, true, motorsConfig
    )

    private var leadMotorPosition: StatusSignal<Angle> = leadMotorController.position
    private var followerMotorPosition: StatusSignal<Angle> = followerMotorController.position
    private var leadMotorVoltage: StatusSignal<Voltage> = leadMotorController.motorVoltage
    private var followerMotorVoltage: StatusSignal<Voltage> = followerMotorController.motorVoltage
    private var leadMotorSupplyCurrent: StatusSignal<Current> = leadMotorController.supplyCurrent
    private var followerMotorSupplyCurrent: StatusSignal<Current> = followerMotorController.supplyCurrent
    private var leadMotorTemperature: StatusSignal<Temperature> = leadMotorController.deviceTemp
    private var followerMotorTemperature: StatusSignal<Temperature> = followerMotorController.deviceTemp

    private var targetAngularDisplacement = 0.0.degrees

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

    override fun updateInputs(inputs: ElevatorIOInputs) {
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

        inputs.elevatorTargetDisplacement = config.sprocket.angularDisplacementToLinearDisplacement(targetAngularDisplacement)
        inputs.elevatorDisplacement = config.sprocket.angularDisplacementToLinearDisplacement(leadMotorPosition.value)
    }

    override fun setVoltage(voltage: Voltage) { leadMotorController.setControl(voltageRequest.withOutput(voltage)) }
    override fun setAngularDisplacement(motorDisplacement: Angle) {
        leadMotorController.setControl(motionMagicRequest.withPosition(motorDisplacement))
    }

    override fun setTargetAngle(angle: Angle) { targetAngularDisplacement = angle }
    override fun getTargetAngle(): Angle { return targetAngularDisplacement }

    override fun getMotorPosition(): Angle { return leadMotorPosition.value }
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