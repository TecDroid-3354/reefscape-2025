package net.tecdroid.autonomous

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.systems.ArmOrders
import net.tecdroid.systems.ArmPoses
import net.tecdroid.systems.ArmSystem
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.util.seconds
import net.tecdroid.vision.limelight.systems.LimelightController

enum class AutonomousSide(val str: String) {
    Right("Right"),
    Center("Center"),
    Left("Left"),
}

enum class AutonomousDirection(val str: String) {
    ReefToCoralStation("reefToCoralStation"),
    CoralStationToReef("coralStationToReef"),
    BargeToReef("bargeToReef"),
}

enum class AutonomousCoralChoice(val str: String) {
    C1("C1"),
    C2("C2"),
    C3("C3"),
}

class AutoComposer(private val drive: SwerveDrive, private val limelightController: LimelightController, private val armSystem: ArmSystem) {
    private val pathplanner = PathPlannerAutonomous(drive, limelightController, armSystem)
    private val autoChooser = SendableChooser<Command>()

    init {

    }

    fun getRoutine(side: AutonomousSide, direction: AutonomousDirection, coralChoice: AutonomousCoralChoice, llChoice: LimeLightChoice): Command {
        return when (direction) {
            AutonomousDirection.ReefToCoralStation -> SequentialCommandGroup(
                ParallelCommandGroup(
                    pathplanner.getPathFollowingCommand("${coralChoice.str}-${side.str}-${direction.str}").beforeStarting(Commands.waitTime(0.5.seconds)),
                    armSystem.setPoseCommand(ArmPoses.CoralStation.pose, ArmOrders.EJW.order),
                ),

                armSystem.enableIntakeAuto(),
                Commands.waitUntil { armSystem.intake.hasCoral() },
                armSystem.disableIntake()
            )

            AutonomousDirection.CoralStationToReef, AutonomousDirection.BargeToReef -> SequentialCommandGroup(
                (if (direction == AutonomousDirection.BargeToReef)
                    pathplanner.resetPoseAndGetPathFollowingCommand("${coralChoice.str}-${side.str}-${direction.str}")
                        .alongWith(armSystem.setPoseCommand(ArmPoses.L2.pose, ArmOrders.WJE.order))
                else
                     pathplanner.getPathFollowingCommand("${coralChoice.str}-${side.str}-${direction.str}")),

                ParallelCommandGroup(
                    limelightController.alignRobotAllAxis(llChoice, 0.19, 0.0)
                        .until { limelightController.isAtSetPoint(llChoice, 0.19, 0.0) },
                    armSystem.setPoseCommand(ArmPoses.L4.pose, ArmOrders.JEW.order).beforeStarting(Commands.waitTime(0.5.seconds)),
                ),

                Commands.waitTime(0.5.seconds),

                armSystem.enableIntakeAuto(),
                Commands.waitUntil { !armSystem.intake.hasCoral() },
                Commands.waitTime(0.5.seconds),
                armSystem.disableIntake(),
            )
        }
    }

    private fun rightCompleteAuto(): Command {
        return Commands.sequence(
            getRoutine(AutonomousSide.Left, AutonomousDirection.BargeToReef, AutonomousCoralChoice.C1, LimeLightChoice.Right),
            getRoutine(AutonomousSide.Left, AutonomousDirection.ReefToCoralStation, AutonomousCoralChoice.C2, LimeLightChoice.Right),
            getRoutine(AutonomousSide.Left, AutonomousDirection.CoralStationToReef, AutonomousCoralChoice.C2, LimeLightChoice.Right),
            getRoutine(AutonomousSide.Left, AutonomousDirection.ReefToCoralStation, AutonomousCoralChoice.C3, LimeLightChoice.Left),
            getRoutine(AutonomousSide.Left, AutonomousDirection.CoralStationToReef, AutonomousCoralChoice.C3, LimeLightChoice.Left),
        )
    }

    private fun leftCompleteAuto(): Command {
        return Commands.sequence(
            getRoutine(AutonomousSide.Right, AutonomousDirection.BargeToReef, AutonomousCoralChoice.C1, LimeLightChoice.Right),
            getRoutine(AutonomousSide.Right, AutonomousDirection.ReefToCoralStation, AutonomousCoralChoice.C2, LimeLightChoice.Right),
            getRoutine(AutonomousSide.Right, AutonomousDirection.CoralStationToReef, AutonomousCoralChoice.C2, LimeLightChoice.Right),
            getRoutine(AutonomousSide.Right, AutonomousDirection.ReefToCoralStation, AutonomousCoralChoice.C3, LimeLightChoice.Left),
            getRoutine(AutonomousSide.Right, AutonomousDirection.CoralStationToReef, AutonomousCoralChoice.C3, LimeLightChoice.Left),
        )
    }

    private fun centerCompleteAuto(): Command {
        return Commands.sequence(
            getRoutine(AutonomousSide.Center, AutonomousDirection.BargeToReef, AutonomousCoralChoice.C1, LimeLightChoice.Right),
            //getRoutine(AutonomousSide.Center, AutonomousDirection.ReefToCoralStation, AutonomousCoralChoice.C2, LimeLightChoice.Right),
            //getRoutine(AutonomousSide.Center, AutonomousDirection.CoralStationToReef, AutonomousCoralChoice.C2, LimeLightChoice.Right),
            //getRoutine(AutonomousSide.Center, AutonomousDirection.ReefToCoralStation, AutonomousCoralChoice.C3, LimeLightChoice.Left),
            //getRoutine(AutonomousSide.Center, AutonomousDirection.CoralStationToReef, AutonomousCoralChoice.C3, LimeLightChoice.Left),
        )
    }

    val selectedAutonomousRoutine: Command
        get() = autoChooser.selected
}