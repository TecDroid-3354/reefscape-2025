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
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.systems.SwerveSystem
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.seconds


class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveSystem(swerveDriveConfiguration)
    private val pathPlannerAuto = PathPlannerAutonomous(swerve.drive)

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

    var vx = { swerve.drive.maxLinearVelocity * longitudinalRateLimiter.calculate(controller.leftY * 0.85) }
    var vy = { swerve.drive.maxLinearVelocity * transversalRateLimiter.calculate(controller.leftX * 0.85) }
    var vw = { swerve.drive.maxAngularVelocity * angularRateLimiter.calculate(controller.rightX * 0.85) }

    init {
        swerve.drive.heading = 0.0.degrees
    }

    fun autonomousInit() {
        swerve.drive.removeDefaultCommand()
        pathPlannerAuto.publishToShuffleboard("Auto")
    }

    fun teleopInit() {
        swerve.linkReorientationTrigger(controller.start())

        swerve.drive.defaultCommand = Commands.run(
            { swerve.drive.driveFieldOriented(ChassisSpeeds(vx(), vy(), vw()))},
            swerve.drive
        )
    }

    val autonomousCommand: Command
        get() {
            return pathPlannerAuto.getAuto("90 deg to right")
            //return pathPlannerAuto.getAuto("Left auto")
        }

}
