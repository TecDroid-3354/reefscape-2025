package net.tecdroid.core

import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.autonomous.PathPlannerAutonomous
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.seconds


class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveDrive(swerveDriveConfiguration)
    private val pathPlannerAuto = PathPlannerAutonomous(swerve)

    // Autonomous
    private var autoChooser: SendableChooser<Command>? = null

    // Swerve Control
    private val accelerationPeriod = 0.1.seconds
    private val decelerationPeriod = accelerationPeriod

    private val da = accelerationPeriod.asFrequency()
    private val dd = -decelerationPeriod.asFrequency()

    private val longitudinalRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)
    private val transversalRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)
    private val angularRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)

    var vx = { swerve.maxLinearVelocity * longitudinalRateLimiter.calculate(controller.leftY * 0.85) }
    var vy = { swerve.maxLinearVelocity * transversalRateLimiter.calculate(controller.leftX * 0.85) }
    var vw = { swerve.maxAngularVelocity * angularRateLimiter.calculate(controller.rightX * 0.85) }

    init {
        swerve.heading = 0.0.degrees
    }

    fun autonomousInit() {
        swerve.removeDefaultCommand()
        pathPlannerAuto.publishToShuffleboard("Auto")
    }

    fun teleopInit() {
        controller.start().onTrue(swerve.zeroHeadingCommand())

        swerve.defaultCommand = Commands.run(
            { swerve.driveFieldOriented(ChassisSpeeds(vx(), vy(), vw()))},
            swerve
        )

        controller.a().whileTrue(Commands.run(
            { swerve.driveFieldOriented(ChassisSpeeds(1.0, 0.0, 0.0))},
            swerve
        ))
    }

    val autonomousCommand: Command
        get() {
            return pathPlannerAuto.getPath("90 deg to right")
            //return pathPlannerAuto.getAuto("Left auto")
        }

}
