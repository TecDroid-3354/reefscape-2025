package net.tecdroid.core
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import edu.wpi.first.util.datalog.BooleanLogEntry
import edu.wpi.first.util.datalog.DoubleLogEntry
import edu.wpi.first.util.datalog.StringLogEntry
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevator.elevatorConfig
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.subsystems.intake.intakeConfig
import net.tecdroid.subsystems.wrist.wristConfig
import net.tecdroid.systems.ArmOrders
import net.tecdroid.systems.ArmPoses
import net.tecdroid.systems.ArmSystem
import net.tecdroid.systems.SwerveSystem
import net.tecdroid.util.units.degrees


class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    val swerve = SwerveSystem(swerveDriveConfiguration)
    private val arm = ArmSystem(wristConfig, elevatorConfig, elevatorJointConfig, intakeConfig)
    private var isNormalMode = true
    private val pollNormalMode = { isNormalMode }
    private var isLow = false
    private val pollIsLow = { isLow }
    private val makeLow = { Commands.runOnce({ isLow = true }) }
    private val makeHigh = { Commands.runOnce({ isLow = false }) }
    val chooser = AutoChooser()

    val autoFactory = AutoFactory(
        swerve.drive::pose,
        swerve.drive::resetOdometry,
        swerve.drive::followTrajectory,
        true,
        swerve.drive
    )

    init {
        linkPoses()
        loadTrajectories()
        swerve.drive.heading = 0.0.degrees
    }

    fun initial() {
        linkMovement()

    }

    fun always() {
        swerve.always()
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

    fun back2m() = Commands.sequence(
        Commands.runOnce({
            autoFactory.resetOdometry("Back2M")
        }),
        autoFactory.trajectoryCmd("Back2M")
    )

    fun loadTrajectories() {
        chooser.addCmd("Back 2 m", ::back2m)

        SmartDashboard.putData("AutoChooser", chooser)

        RobotModeTriggers.autonomous().whileTrue(chooser.selectedCommandScheduler());
    }
}





