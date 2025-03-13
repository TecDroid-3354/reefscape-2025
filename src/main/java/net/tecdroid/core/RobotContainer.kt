package net.tecdroid.core

import choreo.auto.AutoChooser
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevator.elevatorConfig
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.subsystems.intake.Intake
import net.tecdroid.subsystems.intake.intakeConfig
import net.tecdroid.subsystems.wrist.wristConfig
import net.tecdroid.systems.SwerveSystem
import net.tecdroid.systems.arm.*
import net.tecdroid.util.units.meters
import net.tecdroid.util.units.rotations
import net.tecdroid.util.units.volts
import net.tecdroid.auto.AutoRoutines


class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveSystem(swerveDriveConfiguration)
    private val arm = ArmSystem(wristConfig, elevatorConfig, elevatorJointConfig)
    private val intake = Intake(intakeConfig)

    private val auto = AutoRoutines(swerve.drive, intake, arm)
    private val autoChooser = AutoChooser()

    init {
        val tab = Shuffleboard.getTab("Robot Container")
        tab.add("Arm System", arm)

        swerve.linkControllerSticks(controller)
        swerve.linkReorientationTrigger(controller.start())
        autoDashboard();

        // Limelights
        swerve.alignToRightAprilTagTrigger(Trigger { controller.leftTriggerAxis > 0.0 }, controller)
        swerve.alignToLeftAprilTagTrigger(Trigger { controller.rightTriggerAxis > 0.0 }, controller)

//        controller.back().onTrue(
//            arm.setPoseCommand(
//                ArmPoses.Passive.pose,
//                ArmOrders.WEJ.order
//            )
//        )

        controller.y().onTrue(
            arm.setPoseCommand(
                ArmPoses.L4.pose,
                ArmOrders.JEW.order
            )
        )

        controller.b().onTrue(
            arm.setPoseCommand(
                ArmPoses.L3.pose,
                ArmOrders.JEW.order
            )
        )

        controller.a().onTrue(
            arm.setPoseCommand(
                ArmPoses.L2.pose,
                ArmOrders.JEW.order
            )
        )

        controller.x().onTrue(
            arm.setPoseCommand(
                ArmPoses.CoralStation.pose,
                ArmOrders.EJW.order
            )
        )

        controller.rightBumper().onTrue(intake.setVoltageCommand(10.0.volts)).onFalse(intake.setVoltageCommand(0.0.volts))

        controller.back().onTrue(Commands.runOnce({
            arm.wrist.coast()
            arm.joint.coast()
            arm.elevator.coast()
        }))

            controller.leftBumper().onTrue(Commands.runOnce({
            arm.wrist.brake()
            arm.joint.brake()
            arm.elevator.brake()
        }))

    }

    private fun autoDashboard() {
        // Test paths made for tuning
        autoChooser.addRoutine("run two meters forward routine", auto::runTwoMeters);
        autoChooser.addCmd("run two meters forward cmd", auto::runTwoMetersCMD);

        autoChooser.addRoutine("run two meters backward routine", auto::runMinusTwoMeters);
        autoChooser.addCmd("run two meters backward cmd", auto::runMinusTwoMetersCMD);

        autoChooser.addRoutine("left to right auto routine", auto::leftToRight);
        autoChooser.addCmd("left to right auto cmd", auto::leftToRightCMD);

        autoChooser.addRoutine("right to left auto cmd", auto::rightToLeft);
        autoChooser.addCmd("right to left auto cmd", auto::rightToLeftCMD);

        SmartDashboard.putData(autoChooser);

        // Schedules the selected auto for autonomous
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    val autonomousCommand: Command?
        get() = autoChooser.selectedCommand();
}
