package net.tecdroid.autonomous

import choreo.auto.AutoChooser
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.DriveFeedforwards
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.subsystems.drivetrain.SwerveDrive
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
        PIDConstants(5.0, 0.0, 0.0),
        PIDConstants(5.0, 0.0, 0.0)
    )

    private val autoChooser: SendableChooser<Command>

    init {
        AutoBuilder.configure(
            drive::pose::get,
            drive::pose::set,
            drive::speeds::get,
            { speeds: ChassisSpeeds, _: DriveFeedforwards -> drive.driveRobotRelative(speeds) },
            driveController,
            robotConfig,
            { DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red },
            drive
        )

        autoChooser = AutoBuilder.buildAutoChooser()
    }

    fun registerNamedCommand(name: String, command: Command) {
        NamedCommands.registerCommand(name, command)
    }

    fun getPath(name: String) = try {
        val path = PathPlannerPath.fromPathFile(name)
        Commands.runOnce({ drive.pose = path.pathPoses.first() }).andThen(AutoBuilder.followPath(path))
    } catch (e: Exception) {
        DriverStation.reportError(">>>>> Path Planner Autonomous Error\n" + e.message, false)
        Commands.none()
    }

    fun getAuto(name: String): Command = PathPlannerAuto(autoChooser.selected.name)

    fun publishToShuffleboard(tabName: String) {
        val tab = Shuffleboard.getTab(tabName)
        tab.add("Autonomous Chooser", autoChooser)
    }
}