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
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import net.tecdroid.autonomous.AutoComposer
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevator.Elevator
import net.tecdroid.subsystems.elevator.elevatorConfig
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.subsystems.intake.Intake
import net.tecdroid.subsystems.intake.intakeConfig
import net.tecdroid.subsystems.wrist.Wrist
import net.tecdroid.subsystems.wrist.wristConfig
import net.tecdroid.systems.ArmSystem
import net.tecdroid.systems.PoseCommands
import net.tecdroid.util.degrees
import net.tecdroid.util.seconds
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimelightController


class RobotContainer {
    val controller = CompliantXboxController(driverControllerId)
    val swerve = SwerveDrive(swerveDriveConfiguration)
    val wrist = Wrist(wristConfig)
    val elevator = Elevator(elevatorConfig)
    val elevatorJoint = ElevatorJoint(elevatorJointConfig)
    val intake = Intake(intakeConfig)
    val arm = ArmSystem(this)
    val limelightController = LimelightController(
        swerve,
        { chassisSpeeds -> swerve.driveRobotOriented(chassisSpeeds) },
        { swerve.heading.`in`(Degrees) }, swerve.maxSpeeds.times(0.75)
    )
    val xLimelightToAprilTagSetPoint = 0.215
    val yLimelightToAprilTagSetPoint = 0.045
    private val autoComposer = AutoComposer(swerve, limelightController, arm)

    // Advantage Scope log publisher
    private val robotPosePublisher: StructPublisher<Pose2d> = NetworkTableInstance.getDefault()
        .getStructTopic("RobotPose", Pose2d.struct).publish()

    init {
        limelightController.shuffleboardData()
        swerve.heading = 0.0.degrees

        arm.publishShuffleBoardData()
        arm.assignCommandsToController(controller)
        //assignAutomatedScoringBindings()
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

    fun isLimelightAtSetPoint(choice: LimeLightChoice, xToleranceRange: Double = 0.0): Boolean{
        generateSequence(xToleranceRange) { it - 0.01f }
            .takeWhile { it >= 0.0 }
            .forEach {
                if (limelightController.isAtSetPoint(choice, xLimelightToAprilTagSetPoint + it,
                        if (choice == LimeLightChoice.Right)
                            yLimelightToAprilTagSetPoint else yLimelightToAprilTagSetPoint.unaryMinus())) {
                    return true
                }
            }
        return false
    }

//    fun isLimelightAtSetPoint(choice: LimeLightChoice, xTolerance: Double = 0.0): Boolean {
//        return limelightController.isAtSetPoint(choice, xLimelightToAprilTagSetPoint + xTolerance,
//            if (choice == LimeLightChoice.Right) yLimelightToAprilTagSetPoint else yLimelightToAprilTagSetPoint.unaryMinus()) ||
//                limelightController.isAtSetPoint(choice, xLimelightToAprilTagSetPoint,
//                    if (choice == LimeLightChoice.Right) yLimelightToAprilTagSetPoint else yLimelightToAprilTagSetPoint.unaryMinus())
//    }

    fun robotPeriodic() {
        advantageScopeLogs()
    }

    val autonomousCommand: Command
        get() = autoComposer.selectedAutonomousRoutine

}
