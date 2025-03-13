package net.tecdroid.systems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.constants.leftLimelightName
import net.tecdroid.constants.rightLimelightName
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.SwerveDriveConfig
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver
import net.tecdroid.util.ControlGains
import net.tecdroid.util.pidOutputRange
import net.tecdroid.util.units.radians
import net.tecdroid.util.units.seconds
import net.tecdroid.vision.limelight.Limelight
import net.tecdroid.vision.limelight.LimelightConfig
import kotlin.math.absoluteValue

object LimelightAlignmentHandler {
    enum class LimelightChoice {
        Left, Right
    }

    data class LimelightOffset(
        val longitudinalOffset: Double,
        val transversalOffset: Double,
        val horizontalAngleOffset: Double
    )

    private val rightLimelight = Limelight(LimelightConfig(rightLimelightName))
    private val leftLimelight = Limelight(LimelightConfig(leftLimelightName))

    private val longitudinalGains = ControlGains(
        p = 0.25,
        i = 0.0,
        d = 0.0
    )

    private val transversalGains = ControlGains(
        p = 0.45,
        i = 0.0,
        d = 0.0
    )

    private val thetaGains = ControlGains(
        p = 0.001,
        i = 0.0,
        d = 0.0
    )

    private val leftLongitudinalPid = PIDController(longitudinalGains.p, longitudinalGains.i, longitudinalGains.d)
    private val rightLongitudinalPid = PIDController(longitudinalGains.p, longitudinalGains.i, longitudinalGains.d)

    private val leftTransversalPid = PIDController(transversalGains.p, transversalGains.i, transversalGains.d)
    private val rightTransversalPid = PIDController(transversalGains.p, transversalGains.i, transversalGains.d)

    private val rightThetaPid = PIDController(thetaGains.p, thetaGains.i, thetaGains.d)
    private val leftThetaPid = PIDController(thetaGains.p, thetaGains.i, thetaGains.d)

    private const val velocityFactor = 1

    fun assignLimelightAlignment(choice: LimelightChoice, offset: LimelightOffset, driver: SwerveDriveDriver) {
        val limelight = if (choice == LimelightChoice.Left) leftLimelight else rightLimelight
        val longitudinalPid = if (choice == LimelightChoice.Left) leftLongitudinalPid else rightLongitudinalPid
        val transversalPid = if (choice == LimelightChoice.Left) leftTransversalPid else rightTransversalPid
        val thetaPid = if (choice == LimelightChoice.Left) leftThetaPid else rightThetaPid

        longitudinalPid.setTolerance(0.01)
        transversalPid.setTolerance(0.01)
        thetaPid.setTolerance(0.1)

        driver.longitudinalVelocityFactorSource = {
            if (!limelight.hasTarget || longitudinalPid.atSetpoint()) {
                0.0
            } else {
                val currentLongitudinalOffset = -limelight.offsetFromTarget.z
                longitudinalPid.calculate(currentLongitudinalOffset, offset.longitudinalOffset).coerceIn(pidOutputRange) * velocityFactor
            }
        }

        driver.transversalVelocityFactorSource = {
            if (!limelight.hasTarget || transversalPid.atSetpoint()) {
                0.0
            } else {
                val currentTransversalOffset = limelight.offsetFromTarget.x
                transversalPid.calculate(currentTransversalOffset, offset.transversalOffset).coerceIn(pidOutputRange) * velocityFactor
            }
        }

        driver.angularVelocityFactorSource = {
            if (!limelight.hasTarget || thetaPid.atSetpoint()) {
                0.0
            } else {
                val currentOffset = limelight.getHorizontalAngleOffset.`in`(Degrees)
                thetaPid.calculate(currentOffset, offset.horizontalAngleOffset).coerceIn(pidOutputRange) * velocityFactor
            }
        }
    }


    fun assignLimelightAlignmentCommand(choice: LimelightChoice, offset: LimelightOffset, driver: SwerveDriveDriver) : Command = Commands.runOnce({assignLimelightAlignment(choice, offset, driver)})
}

object ControllerMovementHandler {
    fun assignControllerMovement(controller: CompliantXboxController, driver: SwerveDriveDriver) {
        driver.longitudinalVelocityFactorSource = { controller.leftY * 0.85 }
        driver.transversalVelocityFactorSource = { controller.leftX * 0.85 }
        driver.angularVelocityFactorSource = { controller.rightX * 0.85 }
    }

    fun assignControllerMovementCommand(controller: CompliantXboxController, driver: SwerveDriveDriver) : Command = Commands.runOnce({assignControllerMovement(controller, driver)})
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

    fun linkControllerMovement(controller: CompliantXboxController) {
        ControllerMovementHandler.assignControllerMovement(controller, driver)
        driver.setFieldOriented()
    }

    fun linkLimelightTriggers(leftTrigger: Trigger, rightTrigger: Trigger, controller: CompliantXboxController) {
        driver.setRobotOriented()
        val controllerCommand = { ControllerMovementHandler.assignControllerMovementCommand(controller, driver) }
        leftTrigger.onTrue(LimelightAlignmentHandler.assignLimelightAlignmentCommand(LimelightAlignmentHandler.LimelightChoice.Left, LimelightAlignmentHandler.LimelightOffset(0.2000, 0.0, 0.0), driver)).onFalse(controllerCommand())
        rightTrigger.onTrue(LimelightAlignmentHandler.assignLimelightAlignmentCommand(LimelightAlignmentHandler.LimelightChoice.Right, LimelightAlignmentHandler.LimelightOffset(0.2000, 0.0, 0.0), driver)).onFalse(controllerCommand())
    }

    fun linkReorientationTrigger(trigger: Trigger) {
        trigger.onTrue(drive.setHeadingCommand(0.0.radians).andThen(driver.toggleOrientationCommand()))
    }
}