@file:Suppress("MemberVisibilityCanBePrivate")

package net.tecdroid.systems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.elevator.Elevator
import net.tecdroid.subsystems.elevator.ElevatorConfig
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint
import net.tecdroid.subsystems.elevatorjoint.ElevatorJointConfig
import net.tecdroid.subsystems.intake.Intake
import net.tecdroid.subsystems.intake.IntakeConfig
import net.tecdroid.subsystems.wrist.Wrist
import net.tecdroid.subsystems.wrist.WristConfig
import net.tecdroid.systems.ArmMember.*
import net.tecdroid.util.*
import net.tecdroid.util.stateMachine.Phase
import net.tecdroid.util.stateMachine.StateMachine
import net.tecdroid.util.stateMachine.States
import java.util.Optional

enum class ArmMember {
    ArmWrist, ArmElevator, ArmJoint
}

data class ArmPose(
    var wristPosition: Angle,
    var elevatorDisplacement: Distance,
    var elevatorJointPosition: Angle,
    val targetVoltage: Voltage,
    val targetAngle: Optional<(robotPose: Pose2d) -> Angle>
)
data class ArmOrder(
    val first: ArmMember,
    val second: ArmMember,
    val third: ArmMember
)

enum class ArmPoses(var pose: ArmPose) {
    Passive(ArmPose(
        wristPosition         = 0.021.rotations + 5.0.degrees,
        elevatorDisplacement  = 0.01.meters,
        elevatorJointPosition = 0.25.rotations + 3.5.degrees,
        targetVoltage = 0.0.volts,
        Optional.empty()
    )),

    L1(ArmPose(
        wristPosition         = 0.3528.rotations - 55.0.degrees,
        elevatorDisplacement  = 0.01.meters,
        elevatorJointPosition = 0.263.rotations,
        targetVoltage = 3.5.volts,
        Optional.empty()
    )),

    L2(ArmPose(
        wristPosition         = 0.3528.rotations,
        elevatorDisplacement  = 0.0367.meters + 0.0125.meters,
        elevatorJointPosition = 0.263.rotations,
        targetVoltage = 8.0.volts,
        Optional.empty()
    )),

    L3(ArmPose(
        wristPosition         = 0.3528.rotations - 0.5.degrees,
        elevatorDisplacement  = 0.4281.meters,
        elevatorJointPosition = 0.263.rotations, //elevatorJointPosition = 0.25.rotations + 3.5.degrees,
        targetVoltage = 7.0.volts,
        Optional.empty()
    )),

    L4(ArmPose(
        wristPosition         = 0.3528.rotations,
        elevatorDisplacement  = 1.0283.meters,
        elevatorJointPosition = 0.263.rotations, //0.25.rotations + 3.5.degrees,
        targetVoltage = 8.0.volts,
        Optional.empty()
    )),

    CoralStation(ArmPose(
        wristPosition         = 0.3601.rotations + 2.5.degrees,
        elevatorDisplacement  = 0.01.meters,
        elevatorJointPosition = 0.1622.rotations + 10.5.degrees,
        targetVoltage = 9.0.volts,
        Optional.empty()
        /*Optional.of { pose ->
            0.0.degrees // TODO: Logic
        }*/
    )),

    A1(ArmPose(
        wristPosition         = 0.2798.rotations,
        elevatorDisplacement  = 0.1457.meters,
        elevatorJointPosition = 0.1772.rotations - 1.5.degrees,
        targetVoltage = 12.0.volts,
        Optional.empty()
    )),

    A2(ArmPose(
        wristPosition         = 0.2628.rotations,
        elevatorDisplacement  = 0.4920.meters,
        elevatorJointPosition = 0.1968.rotations - 1.5.degrees,
        targetVoltage = 12.0.volts,
        Optional.empty()
    )),

    Processor(ArmPose(
        wristPosition         = 0.3705.rotations,
        elevatorDisplacement  = 0.0150.meters,
        elevatorJointPosition = 0.0415.rotations + 5.0.degrees,
        targetVoltage = 8.0.volts,
        Optional.of { -90.0.degrees }
    )),

    AlgaeFloorIntake(ArmPose(
        wristPosition         = 0.3705.rotations - 24.0.degrees,
        elevatorDisplacement  = 0.0150.meters,
        elevatorJointPosition = 0.0415.rotations,
        targetVoltage = 8.0.volts,
        Optional.empty()
    )),

    coralFloorIntake(ArmPose(
        wristPosition         = 0.358.rotations - 0.5.degrees,
        elevatorDisplacement  = 0.01.meters,
        elevatorJointPosition = 0.0153.rotations,
        targetVoltage = 9.0.volts,
        Optional.empty()
    )),

    Barge(ArmPose(
        wristPosition         = 0.3476.rotations,
        elevatorDisplacement  = 1.0420.meters,
        elevatorJointPosition = 0.263.rotations,
        targetVoltage = 8.0.volts,
        Optional.empty()
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


class ArmSystem(wristConfig: WristConfig, elevatorConfig: ElevatorConfig, elevatorJointConfig: ElevatorJointConfig,
                intakeConfig: IntakeConfig, val swerve: SwerveDrive, val controller: CompliantXboxController,
                val stateMachine: StateMachine) : Sendable {
    val wrist = Wrist(wristConfig)
    val elevator = Elevator(elevatorConfig)
    val joint = ElevatorJoint(elevatorJointConfig)
    val sensor = DigitalInput(3)
    val intake = Intake(intakeConfig, sensor)

    private var targetVoltage = 0.0.volts

    var isScoring = false

    // To change the position orders according to the position of the entire arm
    private var isLow = { false }
    fun setIsLow(value: Boolean) {
        isLow = { value }
    }

    init {
        wrist.matchRelativeEncodersToAbsoluteEncoders()
        joint.matchRelativeEncodersToAbsoluteEncoders()
    }

    fun setJointAngle(angle: Angle): Command = joint.setAngleCommand(angle)
    fun setElevatorDisplacement(displacement: Distance): Command = elevator.setDisplacementCommand(displacement)
    fun setWristAngle(angle: Angle): Command = wrist.setAngleCommand(angle)

    fun enableIntake(): Command = intake.setVoltageCommand { targetVoltage }
    fun enableIntake(voltage: Double): Command = intake.setVoltageCommand { voltage.volts }
    fun enableOuttake(): Command = intake.setVoltageCommand { -targetVoltage }

    // Passive intake in case of Algae State
    fun disableIntake() : Command = Commands.either(intake.setVoltageCommand { 1.5.volts },
        intake.setVoltageCommand { 0.0.volts }
    , stateMachine.isState(States.AlgaeState))

    private fun getCommandFor(pose: ArmPose, member: ArmMember) : Command = when (member) {
        ArmWrist -> wrist.setAngleCommand(pose.wristPosition).andThen(Commands.waitUntil { wrist.getPositionError() < 50.0.rotations })
        ArmElevator -> elevator.setDisplacementCommand(pose.elevatorDisplacement).andThen(Commands.waitUntil { elevator.getPositionError() < 25.0.rotations })
        ArmJoint -> joint.setAngleCommand(pose.elevatorJointPosition).andThen(Commands.waitUntil { joint.getPositionError() < 25.0.rotations })
    }

    private fun getCommandFor(pose: ArmPose, member: ArmMember, slot: Int) : Command = when (member) {
        ArmWrist -> wrist.setAngleCommand(pose.wristPosition, slot).andThen(Commands.waitUntil { wrist.getPositionError() < 50.0.rotations })
        ArmElevator -> elevator.setDisplacementCommand(pose.elevatorDisplacement).andThen(Commands.waitUntil { elevator.getPositionError() < 25.0.rotations })
        ArmJoint -> joint.setAngleCommand(pose.elevatorJointPosition, slot).andThen(Commands.waitUntil { joint.getPositionError() < 25.0.rotations })
    }

    fun setPoseCommand(pose: ArmPose, order: ArmOrder) : Command {
        return SequentialCommandGroup(
            Commands.runOnce({
                targetVoltage = pose.targetVoltage
                isScoring = when (pose) {
                    ArmPoses.L2.pose, ArmPoses.L3.pose, ArmPoses.L4.pose -> true
                    else -> false
                }
            }),
            getCommandFor(pose, order.first),
            getCommandFor(pose, order.second),
            getCommandFor(pose, order.third))
    }

    fun setPoseCommand(pose: ArmPose, order: ArmOrder, slot: Int) : Command {
        return SequentialCommandGroup(
            Commands.runOnce({
                targetVoltage = pose.targetVoltage
                isScoring = when (pose) {
                    ArmPoses.L2.pose, ArmPoses.L3.pose, ArmPoses.L4.pose -> true
                    else -> false
                }
            }),
            getCommandFor(pose, order.first, slot),
            getCommandFor(pose, order.second, slot),
            getCommandFor(pose, order.third, slot))
    }

    fun setPoseAutoCommand(pose: ArmPose, order: ArmOrder) : Command {
        return SequentialCommandGroup(
            Commands.runOnce({ targetVoltage = pose.targetVoltage }),
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

    fun publishShuffleBoardData() {
        val tab = Shuffleboard.getTab("Driver Tab")
        tab.addBoolean("coral", { getSensorRead() })
        tab.addString("State", { stateMachine.getCurrentState().toString() })
        tab.addDouble("Target Voltage") { targetVoltage.`in`(Volts) }
    }

    fun getSensorRead() : Boolean = !sensor.get()

    fun changeState() {
        when(stateMachine.getCurrentState()) {
            States.CoralState, States.ScoreState, States.IntakeState -> stateMachine.changeState(States.AlgaeState)
            States.AlgaeState -> stateMachine.changeState(States.CoralState)
        }
    }

    // Used to avoid the one command binding of the trigger, and process the logic out of the trigger command
    private fun scheduleCMD(command: Command) = command.schedule()

    private fun assignStatesCommands() {
        // Active passive intake
        States.AlgaeState.config.initialCommand = intake.setVoltageCommand { 1.5.volts }

        // Go to passive position after score a coral
        States.ScoreState.config.endCommand = Commands.waitTime(0.5.seconds)
            .andThen(setPoseCommand(ArmPoses.L2.pose, ArmOrders.EJW.order))
    }

    fun assignCommands(controller: CompliantXboxController) {
        assignStatesCommands()

        // Change state conditions

        // Change to score state when coral is detected
        stateMachine.addCondition({ getSensorRead() }, States.ScoreState, Phase.Teleop)

        // Change to coral state if we are in score state, and we just pull out a coral
        stateMachine.addCondition({ stateMachine.isState(States.ScoreState).invoke() && !getSensorRead() }, States.CoralState, Phase.Teleop)

        controller.povLeft().onTrue(
            Commands.runOnce({changeState()})
        )

        // Y
        controller.y().onTrue(
            Commands.runOnce({
                scheduleCMD(when(stateMachine.getCurrentState()){
                    States.CoralState, States.ScoreState -> setPoseCommand(ArmPoses.L4.pose, ArmOrders.JEW.order)
                    States.IntakeState -> setPoseCommand(ArmPoses.L4.pose, ArmOrders.JEW.order)
                        .andThen({stateMachine.changeState(States.CoralState)})
                    States.AlgaeState -> Commands.sequence(
                        enableIntake(2.0),
                        setPoseCommand(
                            ArmPoses.Barge.pose,
                            ArmOrders.JEW.order
                        ).andThen({ setIsLow(false) }),
                        disableIntake())
                })
            })
        )

        // B
        controller.b().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.CoralState, States.ScoreState -> setPoseCommand(ArmPoses.L3.pose, ArmOrders.JEW.order)
                States.IntakeState -> setPoseCommand(ArmPoses.L3.pose, ArmOrders.JEW.order)
                    .andThen({stateMachine.changeState(States.CoralState)})
                States.AlgaeState -> Commands.sequence(
                    enableIntake(2.0),
                    setPoseCommand(
                        ArmPoses.A2.pose,
                        if (isLow()) ArmOrders.JWE.order else ArmOrders.EWJ.order
                    ).andThen({ setIsLow(false) }),
                    disableIntake())
            })
        }))

        // A
        controller.a().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.CoralState, States.ScoreState -> setPoseCommand(ArmPoses.L2.pose, ArmOrders.EJW.order)
                States.IntakeState -> setPoseCommand(ArmPoses.L2.pose, ArmOrders.EJW.order)
                    .andThen({stateMachine.changeState(States.CoralState)})
                States.AlgaeState -> Commands.sequence(
                    enableIntake(2.0),
                    setPoseCommand(
                        ArmPoses.A1.pose,
                        if (isLow()) ArmOrders.JWE.order else ArmOrders.EWJ.order
                    ).andThen({ setIsLow(false) }),
                    disableIntake())
            })
        }))

        // X
        controller.x().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.CoralState -> setPoseCommand(ArmPoses.CoralStation.pose, ArmOrders.EJW.order)
                    .andThen({ setIsLow(true) })
                    .andThen(Commands.runOnce({ stateMachine.changeState(States.IntakeState)}))

                States.IntakeState, States.ScoreState -> setPoseCommand(ArmPoses.CoralStation.pose, ArmOrders.EJW.order)
                    .andThen({ setIsLow(true) })

                States.AlgaeState -> Commands.sequence(
                    setPoseCommand(
                        ArmPoses.CoralStation.pose,
                        ArmOrders.EJW.order
                    ),
                    Commands.runOnce({ stateMachine.changeState(States.IntakeState)}))
            })
        }))

        // Intake
        controller.rightBumper().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.IntakeState, States.AlgaeState, States.ScoreState -> enableIntake()
                States.CoralState -> Commands.sequence(
                    setPoseCommand(ArmPoses.CoralStation.pose, ArmOrders.EJW.order)
                    .andThen({ setIsLow(true) }),
                    Commands.runOnce({ stateMachine.changeState(States.IntakeState)}),
                    enableIntake()
                )
            })
        })).onFalse(disableIntake())

        // POV down
        controller.povDown().onTrue(
            Commands.sequence(
                enableIntake(3.0),
                setPoseCommand(ArmPoses.AlgaeFloorIntake.pose, ArmOrders.EWJ.order, 1),
                disableIntake(),
                Commands.runOnce({ stateMachine.changeState(States.AlgaeState) })
            )
        )

        // POV right
        controller.povRight().onTrue(
            Commands.sequence(
                enableIntake(3.0),
                setPoseCommand(ArmPoses.Processor.pose, ArmOrders.EWJ.order, 1),
                disableIntake(),
                Commands.runOnce({ stateMachine.changeState(States.AlgaeState) })
            )
        )

        // POV up
        controller.povUp().onTrue(
            Commands.sequence(
                setPoseCommand(ArmPoses.coralFloorIntake.pose, ArmOrders.EWJ.order)
            )
        )

        controller.back().onTrue(setPoseCommand(ArmPoses.L1.pose, ArmOrders.JEW.order))

        controller.leftBumper().onTrue(enableOuttake()).onFalse(disableIntake())
    }

}