package net.tecdroid.systems.arm;

import static net.tecdroid.subsystems.elevatorjoint.ElevatorJointConfigurationKt.getElevatorJointConfig;
import static net.tecdroid.subsystems.wrist.WristConfigurationKt.getWristConfig;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import net.tecdroid.systems.arm.ArmPose;
import net.tecdroid.subsystems.wrist.Wrist;
import net.tecdroid.subsystems.elevator.Elevator;
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

public class ArmController {
    private final Wrist wristSubsystem = new Wrist(getWristConfig());
    private final Elevator elevatorSubsystem = new Elevator();
    private final ElevatorJoint elevatorJointSubsystem = new ElevatorJoint(getElevatorJointConfig());

    private final Supplier<Boolean> elevatorAllowedToMove, wristAllowedToMove;

    public ArmController() {
        elevatorAllowedToMove = () -> elevatorJointSubsystem.getAngle().gte(Degrees.of(60.0));
        wristAllowedToMove = () -> elevatorJointSubsystem.getAngle().gte(Degrees.of(20.0));
    }

    public Command setArmPoseCMD(ArmPose armPose) {
        return Commands.parallel(
                    Commands.run(() -> elevatorJointSubsystem.setTargetAngle(armPose.jointAngle())),

                    Commands.waitUntil(wristAllowedToMove::get).andThen(
                        Commands.run(() -> wristSubsystem.setWristAngle(armPose.wristAngle())
                    ),

                    Commands.waitUntil(elevatorAllowedToMove::get).andThen(
                        Commands.run(() -> elevatorSubsystem.goToPositionCMD(armPose.elevatorExtension()))
                    )
        ));
    }
}