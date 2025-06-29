package net.tecdroid.core

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import net.tecdroid.util.volts

class Robot : TimedRobot() {
    private val container = RobotContainer()
    private val autonomousCommand: Command
        get() = container.autonomousCommand

    override fun robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true)
        container.limelightController.setThrottle(200)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        container.robotPeriodic()
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
        container.limelightController.setThrottle(200)
    }

    override fun disabledPeriodic() {
    }

    override fun disabledExit() {
    }

    override fun autonomousInit() {
        container.limelightController.setThrottle(0)
        container.autonomousInit()
        autonomousCommand.schedule()
    }

    override fun autonomousPeriodic() {
    }

    override fun teleopInit() {
        container.limelightController.setThrottle(0)
        container.teleopInit()
        if (autonomousCommand.isScheduled) autonomousCommand.cancel()
    }

    override fun teleopPeriodic() {
    }

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
