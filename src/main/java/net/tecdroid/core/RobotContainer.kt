package net.tecdroid.core

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.autonomous.AutoComposer
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevator.elevatorConfig
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.subsystems.intake.intakeConfig
import net.tecdroid.subsystems.wrist.wristConfig
import net.tecdroid.systems.ArmSystem
import net.tecdroid.util.degrees
import net.tecdroid.util.seconds
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimelightController


class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveDrive(swerveDriveConfiguration)
    private val limelightController = LimelightController(
        swerve,
        { chassisSpeeds -> swerve.driveRobotOriented(chassisSpeeds) },
        { swerve.heading.`in`(Degrees) }, swerve.maxSpeeds.times(0.75)
    )
    private val arm = ArmSystem(wristConfig, elevatorConfig, elevatorJointConfig, intakeConfig, limelightController)
    private val autoComposer = AutoComposer(swerve, limelightController, arm)

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


    // Advantage Scope log publisher
    private val robotPosePublisher: StructPublisher<Pose2d> = NetworkTableInstance.getDefault()
        .getStructTopic("RobotPose", Pose2d.struct).publish()

    init {
        limelightController.shuffleboardData()
        arm.publishShuffleBoardData()
        swerve.heading = 0.0.degrees

        arm.assignCommandsToController(controller)
    }


    fun autonomousInit() {
        swerve.removeDefaultCommand()
    }

    fun teleopInit() {
        controller.start().onTrue(swerve.zeroHeadingCommand())

        swerve.defaultCommand = Commands.run(
            { swerve.driveFieldOriented(ChassisSpeeds(vx(), vy(), vw())) },
            swerve
        )

        controller.rightTrigger().whileTrue(limelightController.alignRobotAllAxis(LimeLightChoice.Right, 0.175, 0.0))
        controller.leftTrigger().whileTrue(limelightController.alignRobotAllAxis(LimeLightChoice.Left, 0.175, 0.0))

        controller.povRight()
    }

    private fun advantageScopeLogs() {
        robotPosePublisher.set(swerve.pose)

    }

    fun robotPeriodic() {
        advantageScopeLogs()
    }

    val autonomousCommand: Command
        get() = autoComposer.selectedAutonomousRoutine

}
