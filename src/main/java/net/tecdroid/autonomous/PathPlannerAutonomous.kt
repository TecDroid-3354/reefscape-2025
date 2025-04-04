package net.tecdroid.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.util.MatchStatus
import java.io.IOException

class PathPlannerAutonomous(val drive: SwerveDrive) {
    private val robotConfig: RobotConfig = try {
        RobotConfig.fromGUISettings()
    } catch (e: Exception) {
        DriverStation.reportError(
            "Could not initialize Robot Configuration for a Path Planner Autonomous Config",
            e.stackTrace
        )
        throw IOException(e)
    }

    private val driveController = PPHolonomicDriveController(
        PIDConstants(6.0, 0.0, 0.0),
        PIDConstants(18.0, 0.0, 0.0)
    )

    init {
        AutoBuilder.configure(
            drive::pose::get,
            drive::pose::set,
            drive::speeds::get,
            { speeds, _ ->
                drive.driveRobotOriented(speeds)
            },
            driveController,
            robotConfig,
            { false },
            drive
        )
    }

    fun registerNamedCommand(name: String, command: Command) {
        NamedCommands.registerCommand(name, command)
    }

    fun getPath(name: String): PathPlannerPath = try {
        PathPlannerPath.fromPathFile(name)
    } catch (e: Exception) {
        DriverStation.reportError("Path Planner Autonomous Error", false)
        throw e
    }


    fun getPathFollowingCommand(name: String): Command = AutoBuilder.followPath(getPath(name))
    fun getPathFollowingCommand(path: PathPlannerPath): Command = AutoBuilder.followPath(path)

    fun resetPoseAndGetPathFollowingCommand(name: String) : Command {
        val path = getPath(name)
        return resetPoseAndGetPathFollowingCommand(path)
    }

    private fun resetPoseAndGetPathFollowingCommand(path: PathPlannerPath) : Command {
        return Commands.runOnce({
            drive.pose = path.pathPoses.first()
            SmartDashboard.putBoolean("SSS", true)
        }).andThen(getPathFollowingCommand(path))
    }
}
