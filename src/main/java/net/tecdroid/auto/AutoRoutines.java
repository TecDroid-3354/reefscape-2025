package net.tecdroid.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;
import net.tecdroid.subsystems.intake.*;

import static net.tecdroid.subsystems.intake.IntakeConfigurationKt.getIntakeConfig;

public class AutoRoutines {
    private final AutonomousFollower follower;
    private final Intake intake = new Intake(getIntakeConfig());
    public AutoRoutines(SwerveDrive swerveDrive) {
        follower = new AutonomousFollower(swerveDrive);
    }

    public AutoRoutine runTwoMeters() {
        AutoRoutine routine = follower.factory.newRoutine("runTwoMeters");
        AutoTrajectory twoMeters = routine.trajectory("TwoMeters");

        routine.active().onTrue(
                Commands.sequence(
                        twoMeters.resetOdometry(),
                        twoMeters.cmd()
                )
        );

        return routine;
    }

    public AutoRoutine runMinusTwoMeters() {
        AutoRoutine routine = follower.factory.newRoutine("runMinusTwoMeters");
        AutoTrajectory twoMeters = routine.trajectory("NegativeTwoMeters");

        routine.active().onTrue(
                Commands.sequence(
                        twoMeters.resetOdometry(),
                        twoMeters.cmd()
                )
        );

        return routine;
    }

    public Command runTwoMeterCMD() {
        return runTwoMeters().cmd();
    }
    public Command runMinusTwoMeterCMD() {
        return runMinusTwoMeters().cmd();
    }



    // Auto routine: left side

    // TODO: IMPORTANT NOTING THESE ARE ONLY FOR TESTS
    // TODO: WHEN CREATING THE COMPLETE AUTO ROUTINE, WE MUST
    // TODO: CREATE A SINGLE SUPERCLASS INSTEAD OF THREE
    private AutoRoutine leftAuto1() {
        AutoRoutine routine = follower.factory.newRoutine("Left Auto: Part 1");
        AutoTrajectory coralOneTrajectory = routine.trajectory("LeftAuto1");

        routine.active().onTrue(
                Commands.sequence(
                        coralOneTrajectory.resetOdometry(),
                        coralOneTrajectory.cmd()
                )
        );

        return routine;
    }

    private AutoRoutine leftAuto2() {
        AutoRoutine routine = follower.factory.newRoutine("Left Auto: Part 2");
        AutoTrajectory coralTwoTrajectory = routine.trajectory("LeftAuto2");

        routine.active().onTrue(
            Commands.sequence(
                coralTwoTrajectory.resetOdometry(),
                coralTwoTrajectory.cmd()
            )
        );

        return routine;
    }

    private AutoRoutine leftAuto3() {
        AutoRoutine routine = follower.factory.newRoutine("Left Auto: Part 3");
        AutoTrajectory coralThreeTrajectory = routine.trajectory("LeftAuto3");

        routine.active().onTrue(
            Commands.sequence(
                coralThreeTrajectory.resetOdometry(),
                coralThreeTrajectory.cmd()
            )
        );

        return routine;
    }

    public Command leftAuto1CMD() {
        return leftAuto1().cmd();
    }

    public Command leftAuto2CMD() {
        return leftAuto2().cmd();
    }

    public Command leftAuto3CMD() {
        return leftAuto3().cmd();
    }




    // Complete auto routine

    public AutoRoutine leftCompleteAuto() {
        AutoRoutine routine = follower.factory.newRoutine("Left Auto: Complete");
        AutoTrajectory firstCoralMovement = routine.trajectory("LeftAuto1");
        AutoTrajectory secondCoralMovement = routine.trajectory("LeftAuto2");
        AutoTrajectory thirdCoralMovement = routine.trajectory("LeftAuto3");

        // Executes when routine starts
        routine.active().onTrue(
                Commands.sequence(
                        firstCoralMovement.resetOdometry(),
                        firstCoralMovement.cmd()
                        // TODO: Limelight logic
                        // TODO: Arm logic — place
                )
        );



        // Second coral
        secondCoralMovement.active().whileTrue(intake.setVoltageCommand(0.10.volts));


        return routine;
    }

    public Command leftAutoRoutineCMD() {
        return leftCompleteAuto().cmd();
    }
}