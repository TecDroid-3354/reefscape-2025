package net.tecdroid.core

import edu.wpi.first.wpilibj2.command.Command
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevator.elevatorConfig
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.subsystems.intake.intakeConfig
import net.tecdroid.subsystems.wrist.wristConfig
import net.tecdroid.systems.SwerveSystem
import net.tecdroid.systems.ArmOrders
import net.tecdroid.systems.ArmPoses
import net.tecdroid.systems.ArmSystem

class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveSystem(swerveDriveConfiguration)
    private val arm = ArmSystem(wristConfig, elevatorConfig, elevatorJointConfig, intakeConfig)

    init {
        linkMovement()
        linkPoses()
    }

    private fun linkMovement() {
        swerve.linkControllerSticks(controller)
        swerve.linkReorientationTrigger(controller.start())
    }

    private fun linkPoses() {
        controller.y().onTrue(
            arm.setPoseCommand(
                ArmPoses.L4.pose,
                ArmOrders.JEW.order
            )
        )

        controller.b().onTrue(
            arm.setPoseCommand(
                ArmPoses.L3.pose,
                ArmOrders.JEW.order
            )
        )

        controller.a().onTrue(
            arm.setPoseCommand(
                ArmPoses.L2.pose,
                ArmOrders.JEW.order
            )
        )

        controller.x().onTrue(
            arm.setPoseCommand(
                ArmPoses.CoralStation.pose,
                ArmOrders.EJW.order
            )
        )

        controller.rightBumper().onTrue(arm.enableIntake()).onFalse(arm.disableIntake())

    }

    val autonomousCommand: Command?
        get() = null

}
