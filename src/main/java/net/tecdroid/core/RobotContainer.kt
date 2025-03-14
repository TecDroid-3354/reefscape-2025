package net.tecdroid.core

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import choreo.auto.AutoChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.auto.AutoRoutines
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevator.elevatorConfig
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.subsystems.intake.intakeConfig
import net.tecdroid.subsystems.wrist.wristConfig
import net.tecdroid.systems.*

class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveSystem(swerveDriveConfiguration)
    private val arm = ArmSystem(wristConfig, elevatorConfig, elevatorJointConfig, intakeConfig)
    private var isNormalMode = true
    private val pollNormalMode = { isNormalMode }
    private var isLow = false
    private val pollIsLow = { isLow }
    private val makeLow = { Commands.runOnce({ isLow = true }) }
    private val makeHigh = { Commands.runOnce({ isLow = false }) }
    private val auto = AutoRoutines(swerve.drive, arm.intake, arm)
    private val autoChooser = AutoChooser()


    init {
        linkMovement()
        linkPoses()
        autoDashboard()
    }

    private fun linkMovement() {
        swerve.linkReorientationTrigger(controller.start())
        swerve.linkControllerMovement(controller)
        swerve.linkLimelightTriggers(controller.leftTrigger(0.5), controller.rightTrigger(0.5), controller)
    }

    private fun linkPoses() {
        controller.povLeft().onTrue(Commands.runOnce({ isNormalMode = !isNormalMode }))

        controller.y().onTrue(
            Commands.either(
                arm.setPoseCommand(
                    ArmPoses.L4.pose,
                    ArmOrders.JEW.order
                ),
                arm.setPoseCommand(
                    ArmPoses.Barge.pose,
                    ArmOrders.JEW.order
                ).andThen(makeHigh()),
                pollNormalMode
            )
        )

        controller.b().onTrue(
            Commands.either(
                arm.setPoseCommand(
                    ArmPoses.L3.pose,
                    ArmOrders.JEW.order
                ),
                arm.setPoseCommand(
                    ArmPoses.A2.pose,
                    if (pollIsLow()) ArmOrders.JWE.order else ArmOrders.EWJ.order
                ).andThen(makeHigh()),
                pollNormalMode
            )
        )

        controller.a().onTrue(
            Commands.either(
                arm.setPoseCommand(
                    ArmPoses.L2.pose,
                    ArmOrders.EJW.order
                ),
                arm.setPoseCommand(
                    ArmPoses.A1.pose,
                    if (pollIsLow()) ArmOrders.JWE.order else ArmOrders.EWJ.order
                ).andThen(makeHigh()),
                pollNormalMode
            )
        )

        controller.x().onTrue(
            Commands.either(
                arm.setPoseCommand(
                    ArmPoses.CoralStation.pose,
                    ArmOrders.EJW.order
                ),
                arm.setPoseCommand(
                    ArmPoses.Processor.pose,
                    ArmOrders.EWJ.order
                ).andThen(makeLow()),
                pollNormalMode
            )
        )

        controller.rightBumper().onTrue(arm.enableIntake()).onFalse(arm.disableIntake())
        controller.leftBumper().onTrue(arm.enableOuttake()).onFalse(arm.disableIntake())

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

        // Real paths
        autoChooser.addRoutine("left auto routine", auto::leftCompleteAuto);
        autoChooser.addCmd("left auto cmd", auto::leftAutoCMD);

        autoChooser.addRoutine("center auto routine", auto::centerCompleteAuto);
        autoChooser.addCmd("center auto cmd", auto::centerAutoCMD);

        autoChooser.addRoutine("right auto routine", auto::rightCompleteAuto);
        autoChooser.addCmd("right auto cmd", auto::rightAutoCMD);

        SmartDashboard.putData(autoChooser);

        // Schedules the selected auto for autonomous
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    /*val autonomousCommand: Command?
        get() = autoChooser.selectedCommand()*/

    val autonomousCommand: Command?


}
