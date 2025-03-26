@file:Suppress("MemberVisibilityCanBePrivate")

package net.tecdroid.systems

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.elevator.Elevator
import net.tecdroid.subsystems.elevator.ElevatorConfig
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint
import net.tecdroid.subsystems.elevatorjoint.ElevatorJointConfig
import net.tecdroid.subsystems.intake.Intake
import net.tecdroid.subsystems.intake.IntakeConfig
import net.tecdroid.subsystems.wrist.Wrist
import net.tecdroid.subsystems.wrist.WristConfig
import net.tecdroid.systems.ArmMember.*
import net.tecdroid.util.degrees
import net.tecdroid.util.meters
import net.tecdroid.util.rotations
import net.tecdroid.util.volts

enum class ArmMember {
    ArmWrist, ArmElevator, ArmJoint
}

data class ArmPose(
    var wristPosition: Angle,
    var elevatorDisplacement: Distance,
    var elevatorJointPosition: Angle,
    val targetVoltage: Voltage
)
data class ArmOrder(
    val first: ArmMember,
    val second: ArmMember,
    val third: ArmMember
)

enum class ArmPoses(var pose: ArmPose) {
    Passive(ArmPose(
        wristPosition         = 0.17.rotations,
        elevatorDisplacement  = 0.01.meters,
        elevatorJointPosition = 0.26.rotations,
        targetVoltage = 0.0.volts
    )),

    L2(ArmPose(
        wristPosition         = 0.3528.rotations,
        elevatorDisplacement  = 0.0367.meters,
        elevatorJointPosition = 0.2615.rotations,
        targetVoltage = 10.0.volts
    )),

    L3(ArmPose(
        wristPosition         = 0.3528.rotations,
        elevatorDisplacement  = 0.4631.meters,
        elevatorJointPosition = 0.2615.rotations,
        targetVoltage = 10.0.volts
    )),

    L4(ArmPose(
        wristPosition         = 0.3390.rotations,
        elevatorDisplacement  = 1.0233.meters,
        elevatorJointPosition = 0.2610.rotations,
        targetVoltage = 10.0.volts
    )),

    CoralStation(ArmPose(
        wristPosition         = 0.3601.rotations,
        elevatorDisplacement  = 0.01.meters,
        elevatorJointPosition = 0.1822.rotations,
        targetVoltage = 8.0.volts
    )),

    A1(ArmPose(
        wristPosition         = 0.2798.rotations,
        elevatorDisplacement  = 0.1457.meters,
        elevatorJointPosition = 0.1972.rotations,
        targetVoltage = 12.0.volts
    )),

    A2(ArmPose(
        wristPosition         = 0.2628.rotations,
        elevatorDisplacement  = 0.4920.meters,
        elevatorJointPosition = 0.2168.rotations,
        targetVoltage = 12.0.volts
    )),

    Processor(ArmPose(
        wristPosition         = 0.3705.rotations,
        elevatorDisplacement  = 0.0150.meters,
        elevatorJointPosition = 0.0615.rotations,
        targetVoltage = 8.0.volts
    )),

    AlgaeFloorIntake(ArmPose(
        wristPosition         = 0.3705.rotations,
        elevatorDisplacement  = 0.0150.meters,
        elevatorJointPosition = 0.0315.rotations,
        targetVoltage = 8.0.volts
    )),

    Barge(ArmPose(
        wristPosition         = 0.3476.rotations,
        elevatorDisplacement  = 1.035.meters,
        elevatorJointPosition = 0.258.rotations,
        targetVoltage = 11.9.volts
    ))
}

enum class ArmOrders(val order: ArmOrder) {
    JEW(ArmOrder(ArmJoint, ArmElevator, ArmWrist)),
    JWE(ArmOrder(ArmJoint, ArmWrist, ArmElevator)),
    EWJ(ArmOrder(ArmElevator, ArmWrist, ArmJoint)),
    EJW(ArmOrder(ArmElevator, ArmJoint, ArmWrist)),
    WEJ(ArmOrder(ArmWrist, ArmElevator, ArmJoint)),
    WJE(ArmOrder(ArmWrist, ArmJoint, ArmElevator))
}

class ArmSystem(wristConfig: WristConfig, elevatorConfig: ElevatorConfig, elevatorJointConfig: ElevatorJointConfig, intakeConfig: IntakeConfig) : Sendable {
    val wrist = Wrist(wristConfig)
    val elevator = Elevator(elevatorConfig)
    val joint = ElevatorJoint(elevatorJointConfig)
    val intake = Intake(intakeConfig)
    var interactiveWristPose = ArmPoses.L2.pose.wristPosition
        set(value) {field = value}

    var interactiveElevatorDisplacement = ArmPoses.L2.pose.elevatorDisplacement
        set(value) {field = value}
    var interactiveJointPose = ArmPoses.L2.pose.elevatorJointPosition
        set(value) {field = value}

    private var targetVoltage = 0.0.volts

    init {
        wrist.matchRelativeEncodersToAbsoluteEncoders()
        joint.matchRelativeEncodersToAbsoluteEncoders()
    }

    fun setJointAngle(angle: Angle) : Command = joint.setAngleCommand(angle)
    fun setElevatorDisplacement(displacement: Distance) : Command = elevator.setDisplacementCommand(displacement)
    fun setWristAngle(angle: Angle) : Command = wrist.setAngleCommand(angle)

    fun enableIntake() : Command = intake.setVoltageCommand(targetVoltage)
    fun enableOuttake() : Command = intake.setVoltageCommand(-targetVoltage)
    fun disableIntake() : Command = intake.setVoltageCommand((0.0).volts)

    private fun getCommandFor(pose: ArmPose, member: ArmMember) : Command = when (member) {
        ArmWrist -> wrist.setAngleCommand(pose.wristPosition).andThen(Commands.waitUntil { wrist.getPositionError() < 50.0.rotations })
        ArmElevator -> elevator.setDisplacementCommand(pose.elevatorDisplacement).andThen(Commands.waitUntil { elevator.getPositionError() < 25.0.rotations })
        ArmJoint -> joint.setAngleCommand(pose.elevatorJointPosition).andThen(Commands.waitUntil { joint.getPositionError() < 25.0.rotations })
    }

    fun setPoseCommand(pose: ArmPose, order: ArmOrder) : Command {
        targetVoltage = pose.targetVoltage

        return SequentialCommandGroup(
            getCommandFor(pose, order.first),
            getCommandFor(pose, order.second),
            getCommandFor(pose, order.third),
        )
    }

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Elevator Error (Rotations)", { elevator.getPositionError().`in`(Rotations) }) {}
            addDoubleProperty("Joint Error (Rotations)", { joint.getPositionError().`in`(Rotations) }) {}
            addDoubleProperty("Wrist Error (Rotations)", { wrist.getPositionError().`in`(Rotations) }) {}
            addDoubleProperty("Elevator Displacement (Meters)", { elevator.displacement.`in`(Meters) }) {}
            addDoubleProperty("Joint Position (Rotations)", { joint.angle.`in`(Rotations) }) {}
            addDoubleProperty("Wrist Position (Rotations)", { wrist.angle.`in`(Rotations) }) {}
        }
    }

    fun assignCommandsToController(controller: CompliantXboxController) {
        controller.rightBumper().onTrue(enableIntake()).onFalse(disableIntake())
        controller.leftBumper().onTrue(enableOuttake()).onFalse(disableIntake())

        controller.y().onTrue(setPoseCommand(ArmPoses.L4.pose, ArmOrders.JEW.order))
        controller.b().onTrue(setPoseCommand(ArmPoses.L3.pose, ArmOrders.JEW.order))
        controller.a().onTrue(setPoseCommand(ArmPoses.L2.pose, ArmOrders.JEW.order))
        controller.x().onTrue(setPoseCommand(ArmPoses.CoralStation.pose, ArmOrders.EJW.order))
    }

    fun iapWristDelta(delta: Angle): Command {
        return Commands.sequence(
            Commands.runOnce({
                interactiveWristPose = delta + interactiveWristPose
            }),
            setWristAngle(interactiveWristPose))
    }

    fun iapJointDelta(delta: Angle): Command {
        return Commands.sequence(
            Commands.runOnce({
                interactiveJointPose = delta + interactiveJointPose
            }),
            setJointAngle(interactiveJointPose))
    }

    fun iapElevatorDelta(delta: Distance): Command {
        return Commands.sequence(
            Commands.runOnce({
                interactiveElevatorDisplacement = delta + interactiveElevatorDisplacement
            }),
            setElevatorDisplacement(interactiveElevatorDisplacement))
    }

    fun breakMode(): Command {
        return Commands.sequence(
            wrist.brake(),
            elevator.brake(),
            joint.brake()
        )
    }

    fun coastMode(): Command {
        return Commands.sequence(
            wrist.coast(),
            elevator.coast(),
            joint.coast()
        )
    }

}