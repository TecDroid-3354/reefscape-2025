package net.tecdroid.core

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
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
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimelightController


class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveDrive(swerveDriveConfiguration)
    private var joystickDriveScalar = 0.85
    private var joystickDriveAngularScalar = 0.6
    private val arm = ArmSystem(wristConfig, elevatorConfig, elevatorJointConfig, intakeConfig, swerve, controller)
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
        swerve.heading = 0.0.degrees

        arm.publishShuffleBoardData()
        arm.assignCommandsToController(controller)
    }


    fun autonomousInit() {
        swerve.removeDefaultCommand()
    }

    fun teleopInit() {
        controller.start().onTrue(swerve.zeroHeadingCommand())

        swerve.defaultCommand = Commands.run(
            {
                val vx = MathUtil.applyDeadband(controller.leftY, 0.05) * joystickDriveScalar
                val vy = MathUtil.applyDeadband(controller.leftX, 0.05) * joystickDriveScalar
                val vw = MathUtil.applyDeadband(controller.rightX, 0.05) * joystickDriveAngularScalar

                val targetXVelocity = swerve.maxLinearVelocity * vx
                val targetYVelocity = swerve.maxLinearVelocity * vy
                val targetAngularVelocity = swerve.maxAngularVelocity * vw

                SmartDashboard.putNumber("!x", targetAngularVelocity.`in`(DegreesPerSecond))
                SmartDashboard.putNumber("!c", controller.rightX)

                swerve.driveFieldOriented(ChassisSpeeds(targetXVelocity, targetYVelocity, targetAngularVelocity))
            },
            swerve
        )

        controller.rightTrigger().whileTrue(limelightController.alignRobotAllAxis(LimeLightChoice.Right, 0.215, 0.045))
        controller.leftTrigger().whileTrue(limelightController.alignRobotAllAxis(LimeLightChoice.Left, 0.215, -0.045))

        controller.povUp().onTrue(
            if (joystickDriveScalar < 0.85 ) {
                InstantCommand({ joystickDriveScalar += 0.05 })
            } else {
                Commands.none()
            })
        controller.povDown().onTrue(
            if (joystickDriveScalar > 0.2) {
                InstantCommand({ joystickDriveScalar -= 0.05 })
            } else {
                Commands.none()
            }
        )
        controller.povRight().onTrue(
            if (joystickDriveAngularScalar < 0.6) {
                InstantCommand({ joystickDriveAngularScalar += 0.05 })
            } else {
                Commands.none()
            }
        )
        controller.povLeft().onTrue(
            if (joystickDriveAngularScalar > 0.2) {
                InstantCommand({ joystickDriveAngularScalar -= 0.05 })
            } else {
                Commands.none()
            }
        )
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
