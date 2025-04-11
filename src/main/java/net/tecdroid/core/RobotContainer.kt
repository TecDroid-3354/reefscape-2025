package net.tecdroid.core

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
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
import net.tecdroid.util.*
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimelightController


class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveDrive(swerveDriveConfiguration)
    private val arm = ArmSystem(wristConfig, elevatorConfig, elevatorJointConfig, intakeConfig)
    private val limelightController = LimelightController(
        swerve,
        { chassisSpeeds -> swerve.driveRobotOriented(chassisSpeeds) },
        { swerve.heading.`in`(Degrees) }, swerve.maxSpeeds.times(0.75)
    )
    private val autoComposer = AutoComposer(swerve, limelightController, arm)

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
            {
                val vx = MathUtil.applyDeadband(controller.leftY, 0.05) * 0.85
                val vy = MathUtil.applyDeadband(controller.leftX, 0.05) * 0.85
                val vw = MathUtil.applyDeadband(controller.rightX, 0.05) * 0.85

                val targetXVelocity = swerve.maxLinearVelocity * vx
                val targetYVelocity = swerve.maxLinearVelocity * vy
                val targetAngularVelocity = swerve.maxAngularVelocity * vw

                swerve.driveFieldOriented(ChassisSpeeds(targetXVelocity, targetYVelocity, targetAngularVelocity))
            },
            swerve
        )

        controller.rightTrigger().whileTrue(limelightController.alignRobotAllAxis(LimeLightChoice.Right, 0.175, 0.0))
        controller.leftTrigger().whileTrue(limelightController.alignRobotAllAxis(LimeLightChoice.Left, 0.175, 0.0))
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
