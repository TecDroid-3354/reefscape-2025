package net.tecdroid.systems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.constants.leftLimelightName
import net.tecdroid.constants.rightLimelightName
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.SwerveDriveConfig
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver
import net.tecdroid.util.ControlGains
import net.tecdroid.util.units.radians
import net.tecdroid.util.units.seconds
import net.tecdroid.vision.limelight.Limelight
import net.tecdroid.vision.limelight.LimelightConfig

object LimelightAlignmentHandler {
    enum class LimelightChoice {
        Left, Right
    }

    private val rightLimelight = Limelight(LimelightConfig(rightLimelightName))
    private val leftLimelight = Limelight(LimelightConfig(leftLimelightName))

    private val longitudinalGains = ControlGains(
        p = 0.0,
        i = 0.0,
        d = 0.0
    )

    private val transversalGains = ControlGains(
        p = 0.0,
        i = 0.0,
        d = 0.0
    )

    private val leftLongitudinalPid = PIDController(longitudinalGains.p, longitudinalGains.i, longitudinalGains.d)
    private val rightLongitudinalPid = PIDController(longitudinalGains.p, longitudinalGains.i, longitudinalGains.d)

    private val leftTransversalPid = PIDController(transversalGains.p, transversalGains.i, transversalGains.d)
    private val rightTransversalPid = PIDController(transversalGains.p, transversalGains.i, transversalGains.d)

    private val velocityFactor = 0.75

    fun transversalAlignment(limelight: Limelight) {

    }
}

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

    fun linkReorientationTrigger(trigger: Trigger) {
        trigger.onTrue(drive.setHeadingCommand(0.0.radians).andThen(driver.toggleOrientationCommand()))
    }
}