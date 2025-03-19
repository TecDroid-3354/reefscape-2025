package net.tecdroid.core

import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import net.tecdroid.util.units.degrees

class Robot : TimedRobot() {

    private val container = RobotContainer()

    val autonomousCommand: Command
        get() = container.getAutonomousCommand()

    override fun robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun disabledPeriodic() {
    }

    override fun disabledExit() {
    }

    override fun autonomousInit() {
        container.setAuto()
        autonomousCommand.schedule()
    }

    override fun autonomousPeriodic() {
    }

    override fun teleopInit() {
        container.initial()
        SmartDashboard.putData("VX: ") { container.vx() }
        SmartDashboard.putData("VY: ") { container.vy() }
        SmartDashboard.putData("VW: ") { container.vw() }

        if (autonomousCommand.isScheduled) autonomousCommand.cancel()
    }

    override fun teleopPeriodic() { }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {
    }

    override fun simulationInit() {
    }

    override fun simulationPeriodic() {
    }
}
