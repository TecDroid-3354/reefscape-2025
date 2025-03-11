package net.tecdroid.systems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.SwerveDriveConfig
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver
import net.tecdroid.util.NumericId
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.radians
import net.tecdroid.util.units.seconds
import kotlin.math.atan2

class SwerveSystem(swerveDriveConfig: SwerveDriveConfig) {
    private val drive = SwerveDrive(
        config = swerveDriveConfig
    )

    private val driver = SwerveDriveDriver(
        maxLinearVelocity = drive.maxLinearVelocity,
        maxAngularVelocity = drive.maxAngularVelocity,
        accelerationPeriod = 0.1.seconds
    )

    private val heading
        get() = drive.heading

    private val cardinalDirectionAlignmentController = PIDController(0.01, 0.0, 0.01)
    private var lastCardinalHeading = 0.0.radians

    init {
        drive.matchRelativeEncodersToAbsoluteEncoders()
        drive.publishToShuffleboard()

        drive.defaultCommand = Commands.run({
            drive.drive(driver.obtainTargetSpeeds(heading))
        }, drive)
    }

    fun linkControllerSticks(controller: CompliantXboxController) {
        driver.longitudinalVelocityFactorSource = { controller.leftY * 0.85 }
        driver.transversalVelocityFactorSource = { controller.leftX * 0.85 }
        driver.angularVelocityFactorSource = { controller.rightX * 0.85 }
    }

    fun linkCardinalDirectionMovement(controller: CompliantXboxController) {
        driver.angularVelocityFactorSource = {
            val targetAngle = atan2(controller.rightY, controller.rightX).radians - 90.0.degrees
            cardinalDirectionAlignmentController.calculate(heading.`in`(Radians), targetAngle.`in`(Radians)) * 0.25
        }
    }

    fun linkOrientationCommand(trigger: Trigger) {
        trigger.onTrue(drive.setHeadingCommand(0.0.radians).andThen(driver.toggleOrientationCommand()))
    }
}