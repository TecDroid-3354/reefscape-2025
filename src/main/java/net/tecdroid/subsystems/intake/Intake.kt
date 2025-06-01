package net.tecdroid.subsystems.intake

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.subsystems.util.generic.TdSubsystem
import org.littletonrobotics.junction.Logger
import net.tecdroid.core.RobotConstants.currentMode

/**
 * Since the IO layer handles the contact with hardware, the intake subsystem is used to communicate with the rest
 * of the robot (Through the Arm System) and to update the IO layer. Once the robot changes from command based
 * to state based
 * (See [Jack In The Bot's 2024 competition code](https://github.com/FRCTeam2910/2024CompetitionRobot-Public/tree/main)
 * for reference), this subsystem must handle those states.
 * @param io The Input Output layer for the subsystem. Any class that implements [IntakeIO] interface is compatible.
 */
class Intake(private val io: IntakeIO) : TdSubsystem("Intake") {
    private val inputs = IntakeIO.IntakeIOInputs() // Should be IntakeIOInputsAutoLogged, but @AutoLog isn't working.

    override val forwardsRunningCondition = { true }    // SysID running condition
    override val backwardsRunningCondition = { true }   // SysID running condition

    override val motorPosition: Angle
        get() = io.getMotorPosition()

    override val motorVelocity: AngularVelocity
        get() = io.getMotorVelocity()

    override val power: Double
        get() = io.getMotorPower()

    /**
     * Called after the primary constructor. In teleop, a coral inside the intake will cause it to stop automatically.
     */
    init {
        (Trigger { io.isSensorTriggered() }).and { DriverStation.isTeleop() }.onTrue(stopCommand())
    }

    /**
     * Every cycle, input data is updated and processed by AdvantageKit's Logger.
     */
    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }

    /**
     * Interacts with the IO layer to set a voltage to the motor.
     * @param voltage The desired motor voltage. this will be clamped within [[-12V, 12V]]
     */
    override fun setVoltage(voltage: Voltage) {
        io.setVoltage(voltage)
    }

    /**
     * Asks the IO layer if the sensor is triggered. If [currentMode] is REAL, this value is automatically negated.
     * If any other robot mode is selected, this value will always be false.
     * @return If the [Intake]'s sensor detects a coral.
     */
    fun hasCoral() = io.isSensorTriggered()
}
