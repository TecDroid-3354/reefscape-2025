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
import net.tecdroid.subsystems.elevator.Elevator
import net.tecdroid.subsystems.elevator.ElevatorConfig
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint
import net.tecdroid.subsystems.elevatorjoint.ElevatorJointConfig
import net.tecdroid.subsystems.intake.Intake
import net.tecdroid.subsystems.intake.IntakeConfig
import net.tecdroid.subsystems.wrist.Wrist
import net.tecdroid.subsystems.wrist.WristConfig
import net.tecdroid.systems.ArmMember.*
import net.tecdroid.util.meters
import net.tecdroid.util.rotations
import net.tecdroid.util.volts

enum class ArmMember {
    ArmWrist, ArmElevator, ArmJoint
}

data class ArmPose(
    val wristPosition: Angle,
    val elevatorDisplacement: Distance,
    val elevatorJointPosition: Angle,
    val targetVoltage: Voltage
)
data class ArmOrder(
    val first: ArmMember,
    val second: ArmMember,
    val third: ArmMember
)

enum class ArmPoses(val pose: ArmPose) {
    Passive(ArmPose(
        wristPosition         = 0.34.rotations,
        elevatorDisplacement  = 0.01.meters,
        elevatorJointPosition = 0.05.rotations,
        targetVoltage = 0.0.volts
    )),

    L2(ArmPose(
        wristPosition         = 0.3528.rotations,
        elevatorDisplacement  = 0.0367.meters,
        elevatorJointPosition = 0.2615.rotations,
        targetVoltage = 10.0.volts
    )),

    L3(ArmPose(
        wristPosition         = 0.3688.rotations,
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
        wristPosition         = 0.3436.rotations,
        elevatorDisplacement  = 0.02345.meters,
        elevatorJointPosition = 0.1891.rotations,
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

    Barge(ArmPose(
        wristPosition         = 0.3476.rotations,
        elevatorDisplacement  = 1.0420.meters,
        elevatorJointPosition = 0.2549.rotations,
        targetVoltage = 8.0.volts
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
    private val wrist = Wrist(wristConfig)
    private val elevator = Elevator(elevatorConfig)
    private val joint = ElevatorJoint(elevatorJointConfig)
     val intake = Intake(intakeConfig)
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
    fun disableIntake() : Command = intake.setVoltageCommand(0.0.volts)

    private fun getCommandFor(pose: ArmPose, member: ArmMember) : Command = when (member) {
        ArmWrist -> wrist.setAngleCommand(pose.wristPosition).andThen(Commands.waitUntil { wrist.getPositionError() < 3.0.rotations })
        ArmElevator -> elevator.setDisplacementCommand(pose.elevatorDisplacement).andThen(Commands.waitUntil { elevator.getPositionError() < 3.0.rotations })
        ArmJoint -> joint.setAngleCommand(pose.elevatorJointPosition).andThen(Commands.waitUntil { joint.getPositionError() < 1.0.rotations })
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

}