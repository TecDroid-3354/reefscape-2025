package net.tecdroid.core

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevator.elevatorConfig
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.subsystems.wrist.wristConfig
import net.tecdroid.systems.SwerveSystem
import net.tecdroid.systems.arm.*
import net.tecdroid.util.units.meters
import net.tecdroid.util.units.rotations

class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveSystem(swerveDriveConfiguration)
    private val arm = ArmSystem(wristConfig, elevatorConfig, elevatorJointConfig)

    init {
        val tab = Shuffleboard.getTab("Robot Container")
        tab.add("Arm System", arm)

        swerve.linkControllerSticks(controller)
        swerve.linkReorientationTrigger(controller.start())

        controller.back().onTrue(
            arm.setPoseCommand(
                ArmPoses.Passive.pose,
                ArmOrders.WEJ.order
            )
        )

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

    }

    val autonomousCommand: Command?
        get() = null

}
