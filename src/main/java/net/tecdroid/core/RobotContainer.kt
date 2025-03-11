package net.tecdroid.core

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.systems.SwerveSystem
import net.tecdroid.util.units.rotations

class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveSystem(swerveDriveConfiguration)
    private val joint = ElevatorJoint(elevatorJointConfig)
    private val routines = joint.createIdentificationRoutine()
    private val tests = routines.createTests()

    init {
        swerve.linkOrientationCommand(controller.start())
        swerve.linkControllerSticks(controller)

        controller.y().onTrue(joint.setAngleCommand(0.255.rotations))
        controller.x().onTrue(joint.setAngleCommand(0.18.rotations))
        controller.b().onTrue(joint.setAngleCommand(0.12.rotations))
        controller.a().onTrue(joint.setAngleCommand(0.01.rotations))

//        controller.y().whileTrue(tests.dynamicForward)
//        controller.a().whileTrue(tests.dynamicBackward)
//        controller.x().whileTrue(tests.quasistaticForward)
//        controller.b().whileTrue(tests.quasistaticBackward)
    }

    val autonomousCommand: Command?
        get() = null

    fun setup() {
//        climber.matchRelativeEncodersToAbsoluteEncoders()
    }
}
