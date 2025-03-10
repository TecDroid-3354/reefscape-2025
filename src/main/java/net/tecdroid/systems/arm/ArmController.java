package net.tecdroid.systems.arm;

import static edu.wpi.first.units.Units.*;

import net.tecdroid.subsystems.elevator.Elevator;
import net.tecdroid.subsystems.wrist.Wrist;
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

public class ArmController {
    private final Wrist wristSubsystem;
    private final Elevator elevatorSubsystem;
    private final ElevatorJoint elevatorJointSubsystem;

    private final Supplier<Boolean> elevatorAllowedToMove, wristAllowedToMove;

    public ArmController(Wrist wristSubsystem, Elevator elevatorSubsystem, ElevatorJoint elevatorJointSubsystem) {
        this.wristSubsystem = wristSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorJointSubsystem = elevatorJointSubsystem;

        elevatorAllowedToMove = () -> elevatorJointSubsystem.getAngle().gte(Degrees.of(60.0));
        wristAllowedToMove = () -> elevatorJointSubsystem.getAngle().gte(Degrees.of(20.0));
    }

    public Command setArmPoseCMD(ArmPose armPose) {
        return Commands.parallel(
                    elevatorJointSubsystem.setTargetAngleCommand(armPose.jointAngle()),

                    Commands.waitUntil(wristAllowedToMove::get).andThen(
                        wristSubsystem.setAngleCommand(armPose.wristAngle())
                    ,

                    Commands.waitUntil(elevatorAllowedToMove::get).andThen(
                        elevatorSubsystem.setTargetDisplacementCommand(armPose.elevatorExtension())
                    )
        ));
    }
}