package net.tecdroid.core

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.BuildInfo
import net.tecdroid.util.volts
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

class Robot : LoggedRobot() {
    private val container = RobotContainer()
    private val autonomousCommand: Command
        get() = container.autonomousCommand

    override fun robotInit() {
        // Metadata to store in every AdvantageKit log file
        Logger.recordMetadata("ProjectName", BuildInfo.MAVEN_NAME)
        Logger.recordMetadata("BuildDate", BuildInfo.BUILD_DATE)
        Logger.recordMetadata("GitSHA", BuildInfo.GIT_SHA)
        Logger.recordMetadata("GitDate", BuildInfo.GIT_DATE)
        Logger.recordMetadata("GitBranch", BuildInfo.GIT_BRANCH)

        when (BuildInfo.DIRTY) { // Whether code changes are commited or not
            0 -> { Logger.recordMetadata("GitDirty", "All changes commited") }
            1 -> { Logger.recordMetadata("GitDirty", "Uncommitted changes") }
            else -> { Logger.recordMetadata("GitDirty", "Unknown") }
        }

        /** FAT32 formatted USB must be connected to roboRIO */
        when (RobotConstants.currentMode) { // Change data receivers based on the current robot mode
            RobotMode.REAL -> { Logger.addDataReceiver(WPILOGWriter()); Logger.addDataReceiver(NT4Publisher()) }
            RobotMode.SIMULATION -> { Logger.addDataReceiver(NT4Publisher()) }
            RobotMode.REPLAY -> {
                setUseTiming(false) // Run as fast as possible
                val logPath = LogFileUtil.findReplayLog() // Replay source path
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_simulation")))
            }
        }

        SignalLogger.enableAutoLogging(false) // Disable CTRE's auto logging.

        Logger.start() // Start AdvantageKit Logger.

        DriverStation.silenceJoystickConnectionWarning(true)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        container.robotPeriodic()
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun disabledPeriodic() {
    }

    override fun disabledExit() {
    }

    override fun autonomousInit() {
        container.autonomousInit()
        autonomousCommand.schedule()
    }

    override fun autonomousPeriodic() {
    }

    override fun teleopInit() {
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
