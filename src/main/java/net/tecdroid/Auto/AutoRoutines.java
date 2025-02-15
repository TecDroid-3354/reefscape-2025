package net.tecdroid.Auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;

public class AutoRoutines {
    private final AutonomousFollower follower;
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

        SmartDashboard.putString("TwoMeters", "Estoy vivo"),

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
}
