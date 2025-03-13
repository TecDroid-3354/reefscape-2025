package net.tecdroid.core

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.LimelightHelpers

class Robot : TimedRobot() {
    private var autonomousCommand: Command? = null

    private val container = RobotContainer()

    override fun robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance()
            .run()
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun disabledPeriodic() {
    }

    override fun disabledExit() {
    }

    override fun autonomousInit() {
        autonomousCommand = container.autonomousCommand

        if (autonomousCommand != null) {
            autonomousCommand!!.schedule()
        }
    }

    override fun autonomousPeriodic() {
    }

    override fun teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand!!.cancel()
        }
    }

    override fun teleopPeriodic() {
    }

    override fun testInit() {
        CommandScheduler.getInstance()
            .cancelAll()
    }

    override fun testPeriodic() {
    }

    override fun simulationInit() {
    }

    override fun simulationPeriodic() {
    }
}
