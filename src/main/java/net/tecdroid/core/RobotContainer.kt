package net.tecdroid.core

import edu.wpi.first.math.proto.System
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevator.Elevator
import net.tecdroid.subsystems.intake.IntakeController
import net.tecdroid.subsystems.intake.IntakeConfiguration.intakeConfig

class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerveDrive = SwerveDrive(swerveDriveConfiguration)
    private val swerveDriver = SwerveDriveDriver(
        swerveDrive.maxLinearVelocity, swerveDrive.maxAngularVelocity, Seconds.of(0.5))

    // Intake test
    private val intake = IntakeController(intakeConfig)

    // Elevator test
    private val elevator = Elevator();

    init {
        configureDrivers()
        configureCommands()
        configureBindings()
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
        controller.x().onTrue(Commands.runOnce({
                                                   swerveDrive.heading = Radians.zero()
                                                   swerveDriver.toggleOrientation()
                                               }))
    }

    val autonomousCommand: Command?
        get() = null

    fun setup() {
        swerveDrive.matchModuleSteeringEncodersToAbsoluteEncoders()
    }

    fun teleopPeriodic() {
        if (controller.y().asBoolean) {
            elevator.moveMotors(0.5)
        } else if (controller.a().asBoolean) {
            elevator.moveMotors(-0.5)
        } else {
            elevator.stopMotors()
        }
        println("RightMotor: " + elevator.rightMotorRot.`in`(Rotations))
        println("LeftMotor: " + elevator.leftMotorRot.`in`(Rotations))
    }
}
