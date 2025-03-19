package net.tecdroid.core

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Hertz
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Command
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
import net.tecdroid.util.units.seconds
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


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

    // Autonomous
    private val autoConfiguration = swerve.drive
    private var autoChooser: SendableChooser<Command>? = null

    // Swerve Control
    private val accelerationPeriod = 0.1.seconds
    private val decelerationPeriod = accelerationPeriod

    private val da = accelerationPeriod.asFrequency()
    private val dd = -decelerationPeriod.asFrequency()

    private val longitudinalRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)
    private val transversalRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)
    private val angularRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)

    var vx = { swerve.drive.maxLinearVelocity * longitudinalRateLimiter.calculate(controller.leftY * 0.85) }
    var vy = { swerve.drive.maxLinearVelocity * transversalRateLimiter.calculate(controller.leftX * 0.85) }
    var vw = { swerve.drive.maxAngularVelocity * angularRateLimiter.calculate(controller.rightX * 0.85) }

    init {
        //linkPoses()
        //loadTrajectories()
        swerve.drive.heading = 0.0.degrees
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    fun initial() {
        linkMovement()
    }

    fun setAuto() {
        autoConfiguration.configurePathPlanner();
        getAutonomousCommand()
    }

    private fun linkMovement() {
        swerve.linkReorientationTrigger(controller.start())
        swerve.linkLimelightTriggers(controller.leftTrigger(0.5), controller.rightTrigger(0.5), this)
        swerve.drive.defaultCommand = Commands.run(
            { swerve.drive.driveFieldOriented(ChassisSpeeds(vx(), vy(), vw()))},
            swerve.drive
        )
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

    /*fun back2m() = Commands.sequence(
        Commands.runOnce({
            autoFactory.resetOdometry("Back2M")
        }),
        autoFactory.trajectoryCmd("Back2M")
    )

    fun loadTrajectories() {
        chooser.addCmd("Back 2 m", ::back2m)

        SmartDashboard.putData("AutoChooser", chooser)

        RobotModeTriggers.autonomous().whileTrue(chooser.selectedCommandScheduler());
    }*/



    fun getAutonomousCommand(): Command {
        // Load the path you want to follow using its name in the GUI
        var path = PathPlannerPath.fromPathFile("Straightforward");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

}
