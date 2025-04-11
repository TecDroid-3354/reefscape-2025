package net.tecdroid.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.systems.ArmOrders
import net.tecdroid.systems.ArmPoses
import net.tecdroid.systems.ArmSystem
import net.tecdroid.util.seconds
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimelightController
import java.io.IOException
import java.util.*

class PathPlannerAutonomous(val drive: SwerveDrive, private val limelightController: LimelightController, private val armSystem: ArmSystem) {
    private val autoChooser = SendableChooser<Command>()

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

    private fun registerNamedCommand(name: String, command: Command) {
        NamedCommands.registerCommand(name, command)
    }

    private fun namedCommandsInit() {
        // Arm
        registerNamedCommand("ArmCoralStationPose",
            armSystem.setPoseCommand(ArmPoses.CoralStation.pose, ArmOrders.EJW.order))

        registerNamedCommand("ArmL4Pose",
            armSystem.setPoseCommand(ArmPoses.L4.pose, ArmOrders.JEW.order))

        registerNamedCommand("ArmL2PoseWJE",
            armSystem.setPoseCommand(ArmPoses.L2.pose, ArmOrders.WJE.order))

        registerNamedCommand("ArmL2PoseJEW",
            armSystem.setPoseCommand(ArmPoses.L2.pose, ArmOrders.WJE.order))

        // Intake
        registerNamedCommand("EnableIntakeUntilHasCoral",
            armSystem.enableIntake().until({ armSystem.intake.hasCoral() }).andThen(armSystem.disableIntake()))

        // Score Commands
        registerNamedCommand("AlignAndScoreRightBranch",
            Commands.sequence(
                ParallelCommandGroup(
                    limelightController.alignRobotAllAxis(LimeLightChoice.Right, 0.19, 0.0)
                        .until { limelightController.isAtSetPoint(LimeLightChoice.Right, 0.19, 0.0) },
                    armSystem.setPoseCommand(ArmPoses.L4.pose, ArmOrders.JEW.order).beforeStarting(Commands.waitTime(0.5.seconds)),
                ),

                Commands.waitTime(0.5.seconds),

                armSystem.enableIntakeAuto(),
                Commands.waitUntil { !armSystem.intake.hasCoral() },
                Commands.waitTime(0.5.seconds),
                armSystem.disableIntake()
            ))

        registerNamedCommand("AlignAndScoreLeftBranch",
            Commands.sequence(
                ParallelCommandGroup(
                    limelightController.alignRobotAllAxis(LimeLightChoice.Left, 0.19, 0.0)
                        .until { limelightController.isAtSetPoint(LimeLightChoice.Left, 0.19, 0.0) },
                    armSystem.setPoseCommand(ArmPoses.L4.pose, ArmOrders.JEW.order).beforeStarting(Commands.waitTime(0.5.seconds)),
                ),

                Commands.waitTime(0.5.seconds),

                armSystem.enableIntakeAuto(),
                Commands.waitUntil { !armSystem.intake.hasCoral() },
                Commands.waitTime(0.5.seconds),
                armSystem.disableIntake()
            ))
    }

    private fun autoChooserOptions() {
        val tab = Shuffleboard.getTab("Driver Tab")
        autoChooser.setDefaultOption("None", Commands.none())

        autoChooser.addOption("Straight Forward", resetPoseAndGetPathFollowingCommand("Straightforward"))

        // Complete autos
        autoChooser.addOption("RightAuto", PathPlannerAuto("Right Auto"))
        autoChooser.addOption("LeftAuto", PathPlannerAuto("Left Auto"))
        autoChooser.addOption("CenterAuto", PathPlannerAuto("Center Auto"))

        tab.add("Autonomous Chooser", autoChooser)
        SmartDashboard.putData("Autonomous Chooser", autoChooser)
    }

    init {
        var alliance = DriverStation.getAlliance()
        
        AutoBuilder.configure(
            drive::pose::get,
            drive::pose::set,
            drive::speeds::get,
            { speeds, _ ->
                drive.driveRobotOriented(speeds)
            },
            driveController,
            robotConfig,
            { if (alliance.isPresent) { alliance.get() == Alliance.Red } else false },
            drive
        )

        namedCommandsInit()
        autoChooserOptions()
    }

    val selectedAutonomousRoutine: Command
        get() = autoChooser.selected

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
