package net.tecdroid.core

import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj2.command.Command
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.climber.Climber
import net.tecdroid.subsystems.climber.climberConfig
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevator.Elevator
import net.tecdroid.subsystems.elevator.elevatorConfig
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.subsystems.intake.Intake
import net.tecdroid.subsystems.intake.intakeConfig
import net.tecdroid.subsystems.wrist.Wrist
import net.tecdroid.subsystems.wrist.wristConfig
import net.tecdroid.util.units.rotations

import net.tecdroid.systems.arm.ArmController
import net.tecdroid.systems.arm.ArmPositions

class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerveDrive = SwerveDrive(swerveDriveConfiguration)
    private val swerveDriver = SwerveDriveDriver(swerveDrive.maxLinearVelocity, swerveDrive.maxAngularVelocity, Seconds.of(0.1))

    private val joint = ElevatorJoint(elevatorJointConfig)
    private val elevator = Elevator(elevatorConfig)
    private val wrist = Wrist(wristConfig)
    private val intake = Intake(intakeConfig)
    private val climber = Climber(climberConfig)
    
    private val armIntegration = ArmController()
    private val armPositions = ArmPositions()

    init {
        publishShuffleboardContents()
        configureDrivers()
        configureCommands()
        configureBindings()
    }

    private fun publishShuffleboardContents() {
        swerveDrive.publishToShuffleboard()
        wrist.publishToShuffleboard()
        joint.publishToShuffleboard()
        elevator.publishToShuffleboard()
        climber.publishToShuffleboard()
    }

    private fun configureDrivers() {
        swerveDriver.longitudinalVelocityFactorSource = { controller.leftY * 0.85 }
        swerveDriver.transversalVelocityFactorSource = { controller.leftX * 0.85 }
        swerveDriver.angularVelocityFactorSource = { controller.rightX * 0.85 }
    }

    private fun configureCommands() {
        swerveDriver.createDefaultCommand(swerveDrive)
    }

    private fun configureBindings() {
        // controller.x().onTrue(swerveDrive.setHeadingCommand(0.0.radians).andThen(swerveDriver.toggleOrientationCommand()).andThen(Commands.print("Toggled Orientation")))
        /*controller.y().onTrue(joint.setAngleCommand(0.26.rotations))
        controller.x().onTrue(joint.setAngleCommand(0.25.rotations))
        controller.b().onTrue(joint.setAngleCommand(0.15.rotations))
        controller.a().onTrue(joint.setAngleCommand(0.05.rotations))*/
        
        controller.a().onTrue(armIntegration.setArmPoseCMD(armPositions.reefL2));
        controller.x().onTrue(armIntegration.setArmPoseCMD(armPositions.reefL3));
        controller.y().onTrue(armIntegration.setArmPoseCMD(armPositions.reefL4));
    }

    val autonomousCommand: Command?
        get() = null

    fun setup() {
        swerveDrive.matchRelativeEncodersToAbsoluteEncoders()
        wrist.matchRelativeEncodersToAbsoluteEncoders()
        joint.matchRelativeEncodersToAbsoluteEncoders()
        climber.matchRelativeEncodersToAbsoluteEncoders()
    }
}
