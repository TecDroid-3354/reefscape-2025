package net.tecdroid.systems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.constants.leftLimelightName
import net.tecdroid.constants.rightLimelightName
import net.tecdroid.core.RobotContainer
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.SwerveDriveConfig
import net.tecdroid.util.ControlGains
import net.tecdroid.safety.pidOutputRange
import net.tecdroid.util.units.radians
import net.tecdroid.util.units.seconds
import net.tecdroid.vision.limelight.Limelight
import net.tecdroid.vision.limelight.LimelightAprilTagDetector
import net.tecdroid.vision.limelight.LimelightConfig
import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.time.times

object LimelightAlignmentHandler {
    enum class LimelightChoice {
        Left, Right
    }

    data class LimelightOffset(
        val longitudinalOffset: Double,
        val transversalOffset: Double,
        val horizontalAngleOffset: Double
    )

    private val rightLimelight = LimelightAprilTagDetector(LimelightConfig(rightLimelightName, Pose3d()))
    private val leftLimelight = LimelightAprilTagDetector(LimelightConfig(leftLimelightName, Pose3d()))

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
        p = 0.0075,
        i = 0.0,
        d = 0.0
    )

    private val leftLongitudinalPid = PIDController(longitudinalGains.p, longitudinalGains.i, longitudinalGains.d)
    private val rightLongitudinalPid = PIDController(longitudinalGains.p, longitudinalGains.i, longitudinalGains.d)

    private val leftTransversalPid = PIDController(transversalGains.p, transversalGains.i, transversalGains.d)
    private val rightTransversalPid = PIDController(transversalGains.p, transversalGains.i, transversalGains.d)

    private val rightThetaPid = PIDController(thetaGains.p, thetaGains.i, thetaGains.d)
    private val leftThetaPid = PIDController(thetaGains.p, thetaGains.i, thetaGains.d)

    val apriltagAngles = mapOf(
        6 to 300 - 180,
        7 to 0,
        8 to 60 - 180,
        9 to 120 - 180,
        10 to 180,
        11 to 240 - 180,
        17 to 60 - 180,
        18 to 0,
        19 to 300 - 180,
        20 to 240 - 180,
        21 to 180,
        22 to 120 - 180,
    )

    init {
        rightThetaPid.enableContinuousInput(0.0, 360.0)
        leftThetaPid.enableContinuousInput(0.0, 360.0)
    }

    fun assignLimelightAlignment(choice: LimelightChoice, offset: LimelightOffset, drive: SwerveDrive, heading: () -> Angle) {
        val limelight = if (choice == LimelightChoice.Left) leftLimelight else rightLimelight
        val longitudinalPid = if (choice == LimelightChoice.Left) leftLongitudinalPid else rightLongitudinalPid
        val transversalPid = if (choice == LimelightChoice.Left) leftTransversalPid else rightTransversalPid
        val thetaPid = if (choice == LimelightChoice.Left) leftThetaPid else rightThetaPid

        val vx: () -> LinearVelocity = {
            if (!limelight.hasTarget) {
                MetersPerSecond.zero()
            } else {
                val currentLongitudinalOffset = limelight.offsetFromTarget.z.absoluteValue
                drive.maxLinearVelocity * longitudinalPid.calculate(
                    currentLongitudinalOffset,
                    offset.longitudinalOffset
                ).coerceIn(pidOutputRange)
            }
        }

        val vy: () -> LinearVelocity = {
            if (!limelight.hasTarget) {
                MetersPerSecond.zero()
            } else {
                val currentTransversalOffset = limelight.offsetFromTarget.x
                drive.maxLinearVelocity * transversalPid.calculate(currentTransversalOffset, offset.transversalOffset)
                    .coerceIn(pidOutputRange) as Double
            }
        }

        val vw: () -> AngularVelocity = {
            if (!limelight.hasTarget) {
                DegreesPerSecond.zero()

            } else {
                val id = limelight.targetId
                val targetAngle = apriltagAngles[id]
                if (id in 7..11 || id in 17 .. 22) thetaPid.calculate(heading().`in`(Degrees), targetAngle!!.toDouble()).coerceIn(
                    pidOutputRange
                ) else 0.0
            }
        }

        val speeds = ChassisSpeeds(vx(), vy(), vw())

        drive.driveCMD(speeds)
    }

    fun assignLimelightAlignmentCommand(choice: LimelightChoice, offset: LimelightOffset, drive: SwerveDrive, thetaSource: () -> Angle) : Command = Commands.runOnce({assignLimelightAlignment(choice, offset, drive, thetaSource)})
}

class SwerveSystem(swerveDriveConfig: SwerveDriveConfig) {
     val drive = SwerveDrive(
        config = swerveDriveConfig
    )


    private val heading
        get() = drive.heading

    init {
        drive.matchRelativeEncodersToAbsoluteEncoders()
        drive.publishToShuffleboard()
    }

    fun linkLimelightTriggers(leftTrigger: Trigger, rightTrigger: Trigger, rc: RobotContainer) {
        leftTrigger.onTrue(LimelightAlignmentHandler.assignLimelightAlignmentCommand(LimelightAlignmentHandler.LimelightChoice.Left, LimelightAlignmentHandler.LimelightOffset(0.19000, 0.0, 0.0), drive, {drive.heading }))
        rightTrigger.onTrue(LimelightAlignmentHandler.assignLimelightAlignmentCommand(LimelightAlignmentHandler.LimelightChoice.Right, LimelightAlignmentHandler.LimelightOffset(0.19000, 0.0, 0.0), drive, { drive.heading }))
    }

    fun linkReorientationTrigger(trigger: Trigger) {
        trigger.onTrue(drive.setHeadingCommand(0.0.radians))
    }
}