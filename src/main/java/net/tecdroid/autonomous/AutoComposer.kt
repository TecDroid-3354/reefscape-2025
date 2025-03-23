package net.tecdroid.autonomous

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.systems.ArmOrders
import net.tecdroid.systems.ArmPoses
import net.tecdroid.systems.ArmSystem
import net.tecdroid.util.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimelightController

class AutoComposer(private val drive: SwerveDrive, private val limelightController: LimelightController, private val armSystem: ArmSystem) {
    private val pathplanner = PathPlannerAutonomous(drive)
    private val autoChooser = SendableChooser<Command>()

    init {
        val tab = Shuffleboard.getTab("Driver Tab")
        autoChooser.setDefaultOption("None", Commands.none())

        autoChooser.addOption("Straight Forward", pathplanner.resetPoseAndGetPathFollowingCommand("Straightforward"))

        // Testing
        autoChooser.addOption("C1 Barge To Reef", c1BargeToReef())
        autoChooser.addOption("C2 Reef to Coral Station", c2ReefToCoralStation())
        autoChooser.addOption("C2 Coral Station to Reef", c2CoralStationToReef())
        autoChooser.addOption("C3 Reef to Coral Station", c3ReefToCoralStation())
        autoChooser.addOption("C3 Coral Station to Reef", c3CoralStationToReef())

        // Complete autos
        autoChooser.addOption("ONLY TEST WHEN ALL PATHS HAVE FINISHED", leftCompleteAuto())

        tab.add("Autonomous Chooser", autoChooser)
    }


    private fun leftCompleteAuto(): Command {
        return Commands.sequence(
            this.c1BargeToReef(),
            this.c2ReefToCoralStation(),
            this.c2CoralStationToReef(),
            this.c3ReefToCoralStation(),
            this.c3CoralStationToReef()
        )
    }


    private fun c1BargeToReef(): Command {
        // ! From starting line to reef (leaving pre-charged coral)
        return Commands.sequence(
            pathplanner.resetPoseAndGetPathFollowingCommand("C1-Left-bargeToReef"),

            // Poner lógica de la limelight
            // limelightController.alignRobotWithSpecificAprilTag(LimeLightChoice.Right, 0.22, -0.56, 9)

            // ✅ Poner lógica del arm
            // armSystem.setPoseCommand(ArmPoses.L4.pose, ArmOrders.JEW.order),
            // ✅ Poner lógica del outtake
            // armSystem.enableIntake().until { !armSystem.intake.hasCoral() }.andThen(armSystem.disableIntake())
        )
    }
    private fun c2ReefToCoralStation(): Command {
        /*val startingPath = pathplanner.getPath("C2-Left-reefToCoralStation")
        drive.pose = startingPath.pathPoses.first()*/

        // ! From reef to coral station (taking second coral)
        return Commands.sequence(
            // TODO: test only after we've assured the arm doesn't kill itself
            /*Commands.parallel(
                armSystem.setPoseCommand(ArmPoses.L2.pose, ArmOrders.EJW.order),
                pathplanner.getPathFollowingCommand(startingPath),
            ),*/

            // TODO IMPORTANT: test each of these individually
            // TODO IMPORTANT: then we can add them in a parallel command
            //armSystem.setPoseCommand(ArmPoses.L2.pose, ArmOrders.EJW.order),
            //pathplanner.getPathFollowingCommand(startingPath)

            pathplanner.resetPoseAndGetPathFollowingCommand("C2-Left-reefToCoralStation"),

            // ✅ Poner lógica del arm
            //armSystem.setPoseCommand(ArmPoses.CoralStation.pose, ArmOrders.EJW.order),
            // ✅ Poner lógica del outtake
            //armSystem.enableIntake().until { !armSystem.intake.hasCoral() }.andThen(armSystem.disableIntake())
        )
    }
    private fun c2CoralStationToReef(): Command {
        val startingPath = pathplanner.getPath("C2-Left-coralStationToReef")
        drive.pose = startingPath.pathPoses.first()

        // ! From coral station to reef (taking and placing the second coral)
        return Commands.sequence(
            // TODO: test only after we've assured the arm doesn't kill itself
            /*Commands.parallel(
                armSystem.setPoseCommand(ArmPoses.L2.pose, ArmOrders.EJW.order),
                pathplanner.getPathFollowingCommand(startingPath),
            ),*/

            // TODO IMPORTANT: test each of these individually
            // TODO IMPORTANT: then we can add them in a parallel command
            //armSystem.setPoseCommand(ArmPoses.L2.pose, ArmOrders.EJW.order),
            pathplanner.getPathFollowingCommand(startingPath)

            // Poner lógica de la limelight
            //limelightController.alignRobotWithSpecificAprilTag(LimeLightChoice.Right, 0.22, -0.56, 9)

            // ✅ Poner lógica del arm
            // armSystem.setPoseCommand(ArmPoses.L4.pose, ArmOrders.JEW.order),
            // ✅ Poner lógica del outtake
            // armSystem.enableIntake().until { !armSystem.intake.hasCoral() }.andThen(armSystem.disableIntake())
        )
    }
    private fun c3ReefToCoralStation(): Command {
        val startingPath = pathplanner.getPath("C3-Left-reefToCoralStation")
        drive.pose = startingPath.pathPoses.first()

        // ! From reef to coral station (taking third coral)
        return Commands.sequence(
            // TODO: test only after we've assured the arm doesn't kill itself
            /*Commands.parallel(
                armSystem.setPoseCommand(ArmPoses.L2.pose, ArmOrders.EJW.order),
                pathplanner.getPathFollowingCommand(startingPath),
            ),*/

            // TODO IMPORTANT: test each of these individually
            // TODO IMPORTANT: then we can add them in a parallel command
            //armSystem.setPoseCommand(ArmPoses.L2.pose, ArmOrders.EJW.order),
            pathplanner.getPathFollowingCommand(startingPath)
            // ✅ Poner lógica del arm
            //armSystem.setPoseCommand(ArmPoses.CoralStation.pose, ArmOrders.EJW.order),
            // ✅ Poner lógica del outtake
            //armSystem.enableIntake().until { !armSystem.intake.hasCoral() }.andThen(armSystem.disableIntake())
        )
    }
    private fun c3CoralStationToReef(): Command {
        val startingPath = pathplanner.getPath("C3-Left-coralStationToReef")
        drive.pose = startingPath.pathPoses.first()

        // ! From coral station to reef (taking and placing the second coral)
        return Commands.sequence(
            // TODO: test only after we've assured the arm doesn't kill itself
            /*Commands.parallel(
                armSystem.setPoseCommand(ArmPoses.L2.pose, ArmOrders.EJW.order),
                pathplanner.getPathFollowingCommand(startingPath),
            ),*/

            // TODO IMPORTANT: test each of these individually
            // TODO IMPORTANT: then we can add them in a parallel command
            //armSystem.setPoseCommand(ArmPoses.L2.pose, ArmOrders.EJW.order),
            pathplanner.getPathFollowingCommand(startingPath),

            // Poner lógica de la limelight
            // limelightController.alignRobotWithSpecificAprilTag(LimeLightChoice.Right, 0.22, -0.56, 9)

            // ✅ Poner lógica del arm
            // armSystem.setPoseCommand(ArmPoses.L4.pose, ArmOrders.JEW.order),
            // ✅ Poner lógica del outtake
            // armSystem.enableIntake().until { !armSystem.intake.hasCoral() }.andThen(armSystem.disableIntake())
        )
    }


    val selectedAutonomousRoutine: Command
        get() = autoChooser.selected
}