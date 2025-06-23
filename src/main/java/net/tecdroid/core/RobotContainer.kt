package net.tecdroid.core

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
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
import net.tecdroid.systems.PoseCommands
import net.tecdroid.util.degrees
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimelightController


class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveDrive(swerveDriveConfiguration)
    private val arm = ArmSystem(wristConfig, elevatorConfig, elevatorJointConfig, intakeConfig, swerve, controller)
    private val limelightController = LimelightController(
        swerve,
        { chassisSpeeds -> swerve.driveRobotOriented(chassisSpeeds) },
        { swerve.heading.`in`(Degrees) }, swerve.maxSpeeds.times(0.75)
    )
    private val xLimelightToAprilTagSetPoint = 0.215
    private val yLimelightToAprilTagSetPoint = 0.045
    private val autoComposer = AutoComposer(swerve, limelightController, arm)

    // Advantage Scope log publisher
    private val robotPosePublisher: StructPublisher<Pose2d> = NetworkTableInstance.getDefault()
        .getStructTopic("RobotPose", Pose2d.struct).publish()

    init {
        limelightController.shuffleboardData()
        swerve.heading = 0.0.degrees

        arm.publishShuffleBoardData()
        arm.assignCommandsToController(controller)
        assignAutomatedScoringBindings()
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

                SmartDashboard.putNumber("!x", targetAngularVelocity.`in`(DegreesPerSecond))
                SmartDashboard.putNumber("!c", controller.rightX)

                swerve.driveFieldOriented(ChassisSpeeds(targetXVelocity, targetYVelocity, targetAngularVelocity))
            },
            swerve
        )

        controller.rightTrigger().whileTrue(limelightController.alignRobotAllAxis(LimeLightChoice.Right,
                xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint))
        controller.leftTrigger().whileTrue(limelightController.alignRobotAllAxis(LimeLightChoice.Left,
            xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint.unaryMinus()))
    }

    private fun advantageScopeLogs() {
        robotPosePublisher.set(swerve.pose)
    }

    private fun assignAutomatedScoringBindings() {
        /*
         * RIGHT LIMELIGHT BINDINGS
         */
        controller.rightTrigger().and(controller.y()).whileTrue(    // L4 BINDING
            Commands.sequence(
                limelightController.alignRobotAllAxis(LimeLightChoice.Right,
                    xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint)
                    .until {
                        limelightController.isAtSetPoint(LimeLightChoice.Right,
                            xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint)
                    },
                arm.setPoseCommand(PoseCommands.L4),
                arm.enableIntake().until { arm.intake.hasCoral().not() }.andThen(arm.disableIntake()),
                arm.setPoseCommand(PoseCommands.CoralStation)
            )
        )
        controller.rightTrigger().and(controller.b()).whileTrue(    // L3 BINDING
            Commands.sequence(
                limelightController.alignRobotAllAxis(LimeLightChoice.Right,
                    xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint)
                    .until {
                        limelightController.isAtSetPoint(LimeLightChoice.Right,
                            xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint)
                    },
                arm.setPoseCommand(PoseCommands.L3),
                arm.enableIntake().until { arm.intake.hasCoral().not() }.andThen(arm.disableIntake()),
                arm.setPoseCommand(PoseCommands.CoralStation)
            )
        )
        controller.rightTrigger().and(controller.a()).whileTrue(    // L2 BINDING
            Commands.sequence(
                limelightController.alignRobotAllAxis(LimeLightChoice.Right,
                    xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint)
                    .until {
                        limelightController.isAtSetPoint(LimeLightChoice.Right,
                            xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint)
                    },
                arm.setPoseCommand(PoseCommands.L2),
                arm.enableIntake().until { arm.intake.hasCoral().not() }.andThen(arm.disableIntake()),
                arm.setPoseCommand(PoseCommands.CoralStation)
            )
        )

        /*
         * LEFT LIMELIGHT BINDINGS
         */
        controller.leftTrigger().and(controller.y()).whileTrue(    // L4 BINDING
            Commands.sequence(
                limelightController.alignRobotAllAxis(LimeLightChoice.Left,
                    xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint.unaryMinus())
                    .until {
                        limelightController.isAtSetPoint(LimeLightChoice.Left,
                            xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint.unaryMinus())
                    },
                arm.setPoseCommand(PoseCommands.L4),
                arm.enableIntake().until { arm.intake.hasCoral().not() }.andThen(arm.disableIntake()),
                arm.setPoseCommand(PoseCommands.CoralStation)
            )
        )
        controller.leftTrigger().and(controller.b()).whileTrue(    // L3 BINDING
            Commands.sequence(
                limelightController.alignRobotAllAxis(LimeLightChoice.Left,
                    xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint.unaryMinus())
                    .until {
                        limelightController.isAtSetPoint(LimeLightChoice.Left,
                            xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint.unaryMinus())
                    },
                arm.setPoseCommand(PoseCommands.L3),
                arm.enableIntake().until { arm.intake.hasCoral().not() }.andThen(arm.disableIntake()),
                arm.setPoseCommand(PoseCommands.CoralStation)
            )
        )
        controller.leftTrigger().and(controller.a()).whileTrue(    // L2 BINDING
            Commands.sequence(
                limelightController.alignRobotAllAxis(LimeLightChoice.Left,
                    xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint.unaryMinus())
                    .until {
                        limelightController.isAtSetPoint(LimeLightChoice.Left,
                            xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint.unaryMinus())
                    },
                arm.setPoseCommand(PoseCommands.L2),
                arm.enableIntake().until { arm.intake.hasCoral().not() }.andThen(arm.disableIntake()),
                arm.setPoseCommand(PoseCommands.CoralStation)
            )
        )
    }

    fun robotPeriodic() {
        advantageScopeLogs()
    }

    val autonomousCommand: Command
        get() = autoComposer.selectedAutonomousRoutine

}
