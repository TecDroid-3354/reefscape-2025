package net.tecdroid.systems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.LimeLightsController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.SwerveDriveConfig
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.radians
import net.tecdroid.util.units.seconds
import net.tecdroid.vision.limelight.Limelight
import net.tecdroid.vision.limelight.LimelightConfig
import kotlin.math.atan2

class SwerveSystem(swerveDriveConfig: SwerveDriveConfig) {
    val xOffPid = PIDController(0.08, 0.0, 0.0)
    val yOffPid = PIDController(0.08, 0.0, 0.0)
    val rOffPid = PIDController(0.01, 0.0, 0.0)

    val drive = SwerveDrive(
        config = swerveDriveConfig
    )

    private val driver = SwerveDriveDriver(
        maxLinearVelocity = drive.maxLinearVelocity,
        maxAngularVelocity = drive.maxAngularVelocity,
        accelerationPeriod = 0.1.seconds
    )

    private val heading
        get() = drive.heading

    val leftLimelight = Limelight(LimelightConfig(name = "limelight-left"))
    val rightLimelight = Limelight(LimelightConfig(name = "limelight-right"))

    init {
        drive.matchRelativeEncodersToAbsoluteEncoders()
        drive.publishToShuffleboard()

        drive.defaultCommand = Commands.run({
            drive.drive(driver.obtainTargetSpeeds(heading))
        }, drive)
    }

    fun linkControllerSticks(controller: CompliantXboxController) {
        driver.longitudinalVelocityFactorSource = { controller.leftY * 0.25 }
        driver.transversalVelocityFactorSource = { controller.leftX * 0.25 }
        driver.angularVelocityFactorSource = { controller.rightX * 0.25 }
    }

    fun linkReorientationTrigger(trigger: Trigger) {
        trigger.onTrue(drive.setHeadingCommand(0.0.radians).andThen(driver.toggleOrientationCommand()))
    }

    fun alignToLimelight(limelight: Limelight) {
        driver.setRobotOriented()

        driver.longitudinalVelocityFactorSource = {
            val pose = limelight.pose
            if (!limelight.hasTarget()) 0.0 else
            xOffPid.calculate(-pose.z, 0.2809).coerceIn(-1.0, 1.0) * 0.8
        }

        driver.transversalVelocityFactorSource = {
            val pose = limelight.pose
            if (!limelight.hasTarget()) 0.0 else
            yOffPid.calculate(pose.x, 0.0).coerceIn(-1.0, 1.0) * 0.8
        }

//        driver.angularVelocityFactorSource = {
//            val offset = limelight.getHorizontalOffset()
//            if (!limelight.hasTarget()) 0.0 else
//            rOffPid.calculate(offset, 0.0).coerceIn(-1.0, 1.0) * 0.4
//        }
    }

    fun publishToShuffleboard() {
        val tab = Shuffleboard.getTab("Driver")
        tab.add("Limelight Right", rightLimelight)
        tab.add("Limelight Left", leftLimelight)
    }

}