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
import net.tecdroid.subsystems.elevatorjoint.ElevatorJointSystemIdentificationRoutine
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.subsystems.wrist.WristSystemIdentificationRoutine
import net.tecdroid.util.units.radians
import net.tecdroid.util.units.rotations
import net.tecdroid.util.units.volts

class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerveDrive = SwerveDrive(swerveDriveConfiguration)
    private val swerveDriver = SwerveDriveDriver(swerveDrive.maxLinearVelocity, swerveDrive.maxAngularVelocity, Seconds.of(0.1))

    // TODO: Arm integration
    private val wrist = Wrist(wristConfig)
    private val intake = Intake(intakeConfig)

    private val wristSysIdRoutine = WristSystemIdentificationRoutine(wrist)
    private val wristTests = wristSysIdRoutine.createTests()

    init {
        publishShuffleboardContents()
        configureDrivers()
        configureCommands()
        configureBindings()
    }

    private fun publishShuffleboardContents() {
        swerveDrive.publishToShuffleboard()

        // Arm lectures:
        wrist.publishToShuffleboard()
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
//        controller.x().onTrue(swerveDrive.setHeadingCommand(0.0.radians).andThen(swerveDriver.toggleOrientationCommand()).andThen(Commands.print("Toggled Orientation")))
        controller.rightBumper().onTrue(intake.setVoltageCommand(6.0.volts)).onFalse(intake.setVoltageCommand(0.0.volts))
        controller.leftBumper().onTrue(intake.setVoltageCommand((-6.0).volts)).onFalse(intake.setVoltageCommand(0.0.volts))

        controller.y().onTrue(wrist.setAngleCommand(0.3.rotations))
        controller.b().onTrue(wrist.setAngleCommand(0.15.rotations))
        controller.a().onTrue(wrist.setAngleCommand(0.0.rotations))
    }

    val autonomousCommand: Command?
        get() = null

    fun setup() {
        swerveDrive.matchRelativeEncodersToAbsoluteEncoders()
        wrist.matchRelativeEncodersToAbsoluteEncoders()
    }
}
