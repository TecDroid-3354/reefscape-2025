package net.tecdroid.core

import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.intake.Intake
import net.tecdroid.subsystems.intake.intakeConfig
import net.tecdroid.subsystems.wrist.Wrist
import net.tecdroid.subsystems.wrist.wristConfig
import net.tecdroid.subsystems.elevator.Elevator
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.util.units.radians

class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerveDrive = SwerveDrive(swerveDriveConfiguration)
    private val swerveDriver = SwerveDriveDriver(swerveDrive.maxLinearVelocity, swerveDrive.maxAngularVelocity, Seconds.of(0.1))

    // TODO: Arm integration
    private val intake = Intake(intakeConfig)
    private val wrist = Wrist(wristConfig)
    private val joint = ElevatorJoint(elevatorJointConfig)
    private val elevator = Elevator() // elevator takes config directly in the constructor

    init {
        publishShuffleboardContents()
        configureDrivers()
        configureCommands()
        configureBindings()

        // TODO: Arm integration
        configureArm()
    }

    private fun publishShuffleboardContents() {
        swerveDrive.publishToShuffleboard()

        // Arm lectures:
        wrist.publishToShuffleboard()
        joint.publishToShuffleboard()
        elevator.publishToShuffleboard()
    }

    private fun configureDrivers() {
        swerveDriver.longitudinalVelocityFactorSource = { controller.leftY * 0.85 }
        swerveDriver.transversalVelocityFactorSource = { controller.leftX * 0.85 }
        swerveDriver.angularVelocityFactorSource = { controller.rightX * 0.85 }
    }

    // TODO: ARM INTEGRATION
    private fun configureArm() {

    }

    private fun configureCommands() {
        swerveDriver.createDefaultCommand(swerveDrive)
    }

    private fun configureBindings() {
        controller.x().onTrue(swerveDrive.setHeadingCommand(0.0.radians).andThen(swerveDriver.toggleOrientationCommand()).andThen(Commands.print("Toggled Orientation")))
    }

    val autonomousCommand: Command?
        get() = null

    fun setup() {
        swerveDrive.matchRelativeEncodersToAbsoluteEncoders()
        wrist.matchRelativeEncodersToAbsoluteEncoders()
        joint.matchRelativeEncodersToAbsoluteEncoders()
    }
}
