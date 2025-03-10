package net.tecdroid.subsystems.elevator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.*
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.constants.subsystemTabName
import net.tecdroid.subsystems.util.generic.IdentifiableSubsystem
import net.tecdroid.subsystems.util.generic.VoltageControlledSubsystem
import net.tecdroid.subsystems.util.identification.GenericSysIdRoutine
import net.tecdroid.util.units.clamp
import net.tecdroid.util.units.radians

class Elevator(private val config: ElevatorConfig) : IdentifiableSubsystem(), Sendable, VoltageControlledSubsystem {
    private val leadMotorController = TalonFX(config.leadMotorControllerId.id)
    private val followerMotorController = TalonFX(config.followerMotorId.id)

    init {
        configureMotorsInterface()
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        leadMotorController.setControl(request)
    }

    fun setTargetDisplacement(displacement: Distance) {
        val targetDisplacement = clamp(config.minimumDisplacement, config.maximumDisplacement, displacement)
        val targetAngle = config.sprocket.linearDisplacementToAngularDisplacement(config.gearRatio.unapply(targetDisplacement))
        SmartDashboard.putNumber("Invop (Rotations)", targetAngle.`in`(Rotations))
        val request = MotionMagicVoltage(targetAngle)
        leadMotorController.setControl(request)
    }

    fun setTargetDisplacementCommand(displacement: Distance): Command = Commands.runOnce({ setTargetDisplacement(displacement) })

    override val power: Double
        get() = leadMotorController.get()

    override val motorPosition: Angle
        get() = leadMotorController.position.value

    override val motorVelocity: AngularVelocity
        get() = leadMotorController.velocity.value

    val displacement: Distance
        get() = config.sprocket.angularDisplacementToLinearDisplacement(config.gearRatio.apply(motorPosition))

    private fun configureMotorsInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.positiveDirection.toInvertedValue())

            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.currentLimit)

            Slot0
                .withKP(config.controlGains.p)
                .withKI(config.controlGains.i)
                .withKD(config.controlGains.d)
                .withKS(config.controlGains.s)
                .withKV(config.controlGains.v)
                .withKA(config.controlGains.a)
                .withKG(config.controlGains.g)

            MotionMagic
                .withMotionMagicCruiseVelocity(config.gearRatio.unapply(config.motionTargets.angularVelocity(config.sprocket)))
                .withMotionMagicAcceleration(config.gearRatio.unapply(config.motionTargets.angularAcceleration(config.sprocket)))
                .withMotionMagicJerk(config.gearRatio.unapply(config.motionTargets.angularJerk(config.sprocket)))
        }


        leadMotorController.clearStickyFaults()
        followerMotorController.clearStickyFaults()

        leadMotorController.configurator.apply(talonConfig)
        followerMotorController.configurator.apply(talonConfig)

        followerMotorController.setControl(Follower(leadMotorController.deviceID, true))
    }

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current Displacement (Meters)", { displacement.`in`(Meters) }, {})
            addDoubleProperty("Inverse Operation (Rotations)", { motorPosition.`in`(Rotations) }, {})
        }
    }

    fun publishToShuffleboard() {
        val tab = Shuffleboard.getTab(subsystemTabName)
        tab.add("Elevator", this)

    }
    fun createIdentificationRoutine() = GenericSysIdRoutine(
        name = "Elevator",
        subsystem = this,
        forwardsRunningCondition = { displacement < config.maximumDisplacement },
        backwardsRunningCondition = { displacement> config.minimumDisplacement }
    )
}

