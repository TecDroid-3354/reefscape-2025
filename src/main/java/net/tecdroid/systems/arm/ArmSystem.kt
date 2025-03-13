package net.tecdroid.systems.arm

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import net.tecdroid.subsystems.elevator.Elevator
import net.tecdroid.subsystems.elevator.ElevatorConfig
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint
import net.tecdroid.subsystems.elevatorjoint.ElevatorJointConfig
import net.tecdroid.subsystems.wrist.Wrist
import net.tecdroid.subsystems.wrist.WristConfig
import net.tecdroid.systems.arm.ArmMember.*
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.meters
import net.tecdroid.util.units.rotations

enum class ArmMember {
    ArmWrist, ArmElevator, ArmJoint
}

data class ArmPose(val wristPosition: Angle, val elevatorDisplacement: Distance, val elevatorJointPosition: Angle)
data class ArmOrder(val first: ArmMember, val second: ArmMember, val third: ArmMember)

enum class ArmPoses(val pose: ArmPose) {
    Passive(ArmPose(
        wristPosition         = 0.34.rotations,
        elevatorDisplacement  = 0.01.meters,
        elevatorJointPosition = 0.05.rotations
    )),

    L2(ArmPose(
        wristPosition         = 0.36.rotations,
        elevatorDisplacement  = 0.01.meters,
        elevatorJointPosition = 0.26223.rotations
    )),

    L3(ArmPose(
        wristPosition         = 0.34.rotations,
        elevatorDisplacement  = 0.3617.meters,
        elevatorJointPosition = 0.26223.rotations
    )),

    L4(ArmPose(
        wristPosition         = 0.3261.rotations,
        elevatorDisplacement  = 1.0055.meters,
        elevatorJointPosition = 0.26223.rotations
    )),

    CoralStation(ArmPose(
        wristPosition         = 0.378.rotations, // new 0.386
        elevatorDisplacement  = 0.01.meters,
        elevatorJointPosition = 0.188.rotations
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

class ArmSystem(wristConfig: WristConfig, elevatorConfig: ElevatorConfig, elevatorJointConfig: ElevatorJointConfig) : Sendable {
    val wrist = Wrist(wristConfig)
    val elevator = Elevator(elevatorConfig)
    val joint = ElevatorJoint(elevatorJointConfig)

    init {
        wrist.matchRelativeEncodersToAbsoluteEncoders()
        joint.matchRelativeEncodersToAbsoluteEncoders()
    }

    fun setJointAngle(angle: Angle) : Command = joint.setAngleCommand(angle)
    fun setElevatorDisplacement(displacement: Distance) : Command = elevator.setDisplacementCommand(displacement)
    fun setWristAngle(angle: Angle) : Command = wrist.setAngleCommand(angle)

    private fun getCommandFor(pose: ArmPose, member: ArmMember) : Command = when (member) {
        ArmWrist -> wrist.setAngleCommand(pose.wristPosition).andThen(Commands.waitUntil { wrist.getPositionError() < 1.0.rotations })
        ArmElevator -> elevator.setDisplacementCommand(pose.elevatorDisplacement).andThen(Commands.waitUntil { elevator.getPositionError() < 1.0.rotations })
        ArmJoint -> joint.setAngleCommand(pose.elevatorJointPosition).andThen(Commands.waitUntil { joint.getPositionError() < 1.0.rotations })
    }

    fun setPoseCommand(pose: ArmPose, order: ArmOrder) : Command = SequentialCommandGroup(
        getCommandFor(pose, order.first),
        getCommandFor(pose, order.second),
        getCommandFor(pose, order.third),
    )

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

}