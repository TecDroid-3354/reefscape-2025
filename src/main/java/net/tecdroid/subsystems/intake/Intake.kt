package net.tecdroid.subsystems.intake

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.subsystems.util.generic.TdSubsystem
import org.littletonrobotics.junction.Logger


/**
 * Since the IO layer handles the contact with hardware, the intake subsystem is used to communicate with the rest
 * of the robot (Through the Arm System) and to update the IO layer. Once the robot changes from command based
 * to state based
 * (See [Jack In The Bot's 2024 competition code](https://github.com/FRCTeam2910/2024CompetitionRobot-Public/tree/main)
 * for reference), this subsystem must handle those states.
 * @param io The Input Output layer for the subsystem. Any class that implements [IntakeIO] interface is compatible.
 */
class Intake(private val io: IntakeIO) : TdSubsystem("Intake") {
    private val inputs = IntakeIO.IntakeIOInputs()
    private val sensor = DigitalInput(3)
    private val trigger = Trigger(sensor::get)

    override val forwardsRunningCondition = { true }
    override val backwardsRunningCondition = { true }

    override val motorPosition: Angle
        get() = io.getMotorPosition()

    override val motorVelocity: AngularVelocity
        get() = io.getMotorVelocity()

    override val power: Double
        get() = io.getMotorPower()

    init {
        (trigger.negate()).and { DriverStation.isTeleop() }.onTrue(stopCommand())
    }

    /**
     * Every cycle, input data is updated and processed by AdvantageKit's Logger.
     */
    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }

    override fun setVoltage(voltage: Voltage) {
        io.setVoltage(voltage)
    }

    fun hasCoral() = !sensor.get()
}
