package net.tecdroid.core

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevator.Elevator
import net.tecdroid.subsystems.elevator.elevatorConfig
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.systems.SwerveSystem
import net.tecdroid.util.units.meters
import net.tecdroid.util.units.rotations

class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveSystem(swerveDriveConfiguration)
    private val joint = ElevatorJoint(elevatorJointConfig)
    private val elevator = Elevator(elevatorConfig)
    private val routines = elevator.createIdentificationRoutine()
    private val tests = routines.createTests()

    init {
        swerve.linkOrientationCommand(controller.start())
        swerve.linkControllerSticks(controller)
    }

    val autonomousCommand: Command?
        get() = null

    fun setup() {
//        climber.matchRelativeEncodersToAbsoluteEncoders()
    }
}
