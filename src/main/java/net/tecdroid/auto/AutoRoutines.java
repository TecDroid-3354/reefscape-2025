package net.tecdroid.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import net.tecdroid.subsystems.intake.*;
//import net.tecdroid.subsystems.limeLight.LimeLightsController;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;
import net.tecdroid.systems.ArmOrders;
import net.tecdroid.systems.ArmPoses;
import net.tecdroid.systems.ArmSystem;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class AutoRoutines {
    private final Intake intake;
    private final ArmSystem armSystem;
    private final AutonomousFollower follower;
    public AutoRoutines(SwerveDrive swerveSubsystem, Intake intake, ArmSystem armSystem) {
        this.intake = intake;
        this.armSystem = armSystem;

        follower = new AutonomousFollower(swerveSubsystem);
    }

    // TEST SEGMENTS
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

    public Command runTwoMetersCMD() {
        return runTwoMeters().cmd();
    }
    public Command runMinusTwoMetersCMD() {
        return runMinusTwoMeters().cmd();
    }

    public AutoRoutine leftToRight() {
        AutoRoutine routine = follower.factory.newRoutine("leftToRight");
        AutoTrajectory cycle = routine.trajectory("ONEMETER90DEGREESMOVEMENT/LEFTTORIGHT90DEGREES21");

        routine.active().onTrue(
                Commands.sequence(
                        cycle.resetOdometry(),
                        cycle.cmd()
                )
        );

        return routine;
    }

    public Command leftToRightCMD() {
        return leftToRight().cmd();
    }

    public AutoRoutine rightToLeft() {
        AutoRoutine routine = follower.factory.newRoutine("leftToRight");
        AutoTrajectory cycle = routine.trajectory("ONEMETER90DEGREESMOVEMENT/RIGHTTOLEFT90DEGREES21");

        routine.active().onTrue(
                Commands.sequence(
                        cycle.resetOdometry(),
                        cycle.cmd()
                )
        );

        return routine;
    }

    public Command rightToLeftCMD() {
        return rightToLeft().cmd();
    }




    // ! Auto trajectories

    // Routine followers

    // TODO: IMPORTANTÍSIMO ———— FINISH THE ROUTINES
    // TODO: IMPORTANTÍSIMO ———— CHECAR ALIGNMENT CON LIMELIGHTS DURANTE ROUTINES

    // TODO (OPTIONAL) — SET A CYCLE FUNCTION THAT ALLOWS CYCLES TO GET OPERATED
    // TODO 4635 ASK ABOUT AUTO
    private AutoRoutine cycle(AutoTrajectory reefToCoralStation, AutoTrajectory coralStationToReef, AutoTrajectory nextCycle) {
        reefToCoralStation.done().onTrue(
                Commands.sequence(
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(intake::hasCoral).andThen(
                                Commands.sequence(
                                        intake.stopCommand(), // intaking at coral station
                                         coralStationToReef.cmd() // going from coral station to reef
                                )
                        )
                )
        );

        coralStationToReef.done().onTrue(
                Commands.sequence(
                        // TODO: AQUÍ PONER LÓGICA DE LIMELIGHTS PARA ALINEARSE CON EL REEF
                        /* limelights.alignInAllAxis(swerveDriver, Degrees.of(0.0), Inches.of(5.0), true),
                        Commands.waitUntil(() -> limelights.isAlignedAtReef(true) || Commands.waitTime(Seconds.of(3))
                                .isFinished()).andThen(intake.setVoltageCommand(Volts.of(10.0))),*/
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand(), // leaving coral from cycle 2
                                //nextCycle.isPresent() ? nextCycle.get().cmd() : null
                                nextCycle.cmd()
                        )
                )
        );

        return null;
    }


    private AutoRoutine routineFollower(
            AutoRoutine routine, AutoTrajectory firstCycleBargeToReef, AutoTrajectory secondCycleReefToCoralStation,
            AutoTrajectory secondCycleCoralStationToReef, AutoTrajectory thirdCycleReefToCoralStation, AutoTrajectory thirdCycleCoralStationToReef) {

        // Executes when routine starts
        routine.active().onTrue(
                Commands.sequence(
                        firstCycleBargeToReef.resetOdometry(),
                        firstCycleBargeToReef.cmd(),
                        // TODO: CHECK THE EVENT NAME
                        Commands.waitUntil(firstCycleBargeToReef.atTime("armPoseEventMarker")).andThen(
                                armSystem.setPoseCommand(ArmPoses.L4.getPose(), ArmOrders.JEW.getOrder())
                        )
                        // TODO: Limelight logic
                )
        );

        // ####################
        // # GENERAL COMMANDS #
        // ####################

        // All coralStation -> reef cycles will automatically set arm to have reef L4 position
        routine.anyActive(secondCycleReefToCoralStation, thirdCycleReefToCoralStation)
                .whileTrue(armSystem.setPoseCommand(ArmPoses.CoralStation.getPose(), ArmOrders.JEW.getOrder()));

        // All reef --> coralStation cycles will automatically set arm to have reef L4 position
        routine.anyActive(secondCycleCoralStationToReef, thirdCycleCoralStationToReef)
                .whileTrue(armSystem.setPoseCommand(ArmPoses.L4.getPose(), ArmOrders.JEW.getOrder()));

        // ! Cycle #2 : Second coral

        firstCycleBargeToReef.done().onTrue(
                Commands.sequence(
                        // TODO: LÓGICA DE LIMELIGHTS PARA ALINEARSE CON REEF
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand(), // step 1: leaving piece from cycle 1 (precharged)
                                secondCycleReefToCoralStation.cmd() // step 2: going from reef to coral station
                        )
                )
        );

        /*secondCycleReefToCoralStation.done().onTrue(
                Commands.sequence(
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(intake::hasCoral).andThen(
                                Commands.sequence(
                                        intake.stopCommand(), // step 3: intaking at coral station
                                        secondCycleCoralStationToReef.cmd() // step 4: going from coral station to reef
                                )
                        )
                )
        );

        secondCycleCoralStationToReef.done().onTrue(
                Commands.sequence(
                        // TODO: AQUÍ PONER LÓGICA DE LIMELIGHTS PARA ALINEARSE CON EL REEF
                        /* limelights.alignInAllAxis(swerveDriver, Degrees.of(0.0), Inches.of(5.0), true),
                        Commands.waitUntil(() -> limelights.isAlignedAtReef(true) || Commands.waitTime(Seconds.of(3))
                                .isFinished()).andThen(intake.setVoltageCommand(Volts.of(10.0))),
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand(), // step 5: leaving coral from cycle 2
                                thirdCycleReefToCoralStation.cmd() // step 6: going from reef to coral station
                        )
                )
        );


        thirdCycleReefToCoralStation.done().onTrue(
                Commands.sequence(
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(intake::hasCoral).andThen(
                                Commands.sequence(
                                        intake.stopCommand(), // step 7: intaking at coral station
                                        thirdCycleCoralStationToReef.cmd() // step 8: going from coral station to reef
                                )
                        )
                )
        );

        thirdCycleCoralStationToReef.done().onTrue(
                Commands.sequence(
                        // TODO: AQUÍ PONER LÓGICA DE LIMELIGHTS PARA ALINEARSE CON EL REEF
                        /* limelights.alignInAllAxis(swerveDriver, Degrees.of(0.0), Inches.of(5.0), true),
                        Commands.waitUntil(() -> limelights.isAlignedAtReef(true) || Commands.waitTime(Seconds.of(3))
                                .isFinished()).andThen(intake.setVoltageCommand(Volts.of(10.0))),
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand() // step 9: leaving coral from cycle 3
                        )
                )
        );*/

        cycle(secondCycleReefToCoralStation, secondCycleCoralStationToReef, thirdCycleReefToCoralStation);
        cycle(thirdCycleReefToCoralStation, thirdCycleCoralStationToReef, null);

        return routine;
    }

    private AutoRoutine routineFollower(
            AutoRoutine routine, AutoTrajectory firstCycleBargeToReef, AutoTrajectory secondCycleReefToCoralStation,
            AutoTrajectory secondCycleCoralStationToReef, AutoTrajectory thirdCycleReefToCoralStation, AutoTrajectory thirdCycleCoralStationToReef,
            AutoTrajectory fourthCycleReefToCoralStation, AutoTrajectory fourthCycleCoralStationToReef) {

        // Executes when routine starts
        routine.active().onTrue(
                Commands.sequence(
                        firstCycleBargeToReef.resetOdometry(),
                        firstCycleBargeToReef.cmd(),
                        // TODO: CHECK THE EVENT NAME
                        Commands.waitUntil(firstCycleBargeToReef.atTime("armPoseEventMarker")).andThen(
                                armSystem.setPoseCommand(ArmPoses.L4.getPose(), ArmOrders.JEW.getOrder())
                        )
                        // TODO: Limelight logic
                )
        );

        // ####################
        // # GENERAL COMMANDS #
        // ####################

        // All coralStation -> reef cycles will automatically set arm to have reef L4 position
        routine.anyActive(secondCycleReefToCoralStation, thirdCycleReefToCoralStation, fourthCycleReefToCoralStation)
                .whileTrue(armSystem.setPoseCommand(ArmPoses.CoralStation.getPose(), ArmOrders.JEW.getOrder()));

        // All reef --> coralStation cycles will automatically set arm to have reef L4 position
        routine.anyActive(secondCycleCoralStationToReef, thirdCycleCoralStationToReef, fourthCycleCoralStationToReef)
                .whileTrue(armSystem.setPoseCommand(ArmPoses.L4.getPose(), ArmOrders.JEW.getOrder()));

        // ! Cycle #2 : Second coral

        firstCycleBargeToReef.done().onTrue(
                Commands.sequence(
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand(), // step 1: leaving piece from cycle 1 (precharged)
                                secondCycleReefToCoralStation.cmd() // step 2: going from reef to coral station
                        )
                )
        );

        cycle(secondCycleReefToCoralStation, secondCycleCoralStationToReef, thirdCycleReefToCoralStation);
        cycle(thirdCycleReefToCoralStation, thirdCycleCoralStationToReef, fourthCycleReefToCoralStation);
        cycle(fourthCycleReefToCoralStation, fourthCycleCoralStationToReef, null);

        return routine;
    }

    // Center auto
    public AutoRoutine centerCompleteAuto() {
        AutoRoutine routine = follower.factory.newRoutine("Center Auto: Complete");
        AutoTrajectory firstCycleBargeToReef = routine.trajectory("ACENTERAUTONOMAutonomousFollowerOUSMOVEMENT/CENTER-CORAL1-BARGETOREEF");
        AutoTrajectory secondCycleReefToCoralStation = routine.trajectory("ACENTERAUTONOMOUSMOVEMENT/CENTER-CORAL2-BREEFTOCORALSTATION");
        AutoTrajectory secondCycleCoralStationToReef = routine.trajectory("ACENTERAUTONOMOUSMOVEMENT/CENTER-CORAL2-CORALSTATIONTOREEF");
        AutoTrajectory thirdCycleReefToCoralStation = routine.trajectory("ACENTERAUTONOMOUSMOVEMENT/CENTER-CORAL3-BREEFTOCORALSTATION");
        AutoTrajectory thirdCycleCoralStationToReef = routine.trajectory("ACENTERAUTONOMOUSMOVEMENT/CENTER-CORAL3-CORALSTATIONTOREEF");
        AutoTrajectory fourthCycleReefToCoralStation = routine.trajectory("ACENTERAUTONOMOUSMOVEMENT/CENTER-CORAL4-BREEFTOCORALSTATION");
        AutoTrajectory fourthCycleCoralStationToReef = routine.trajectory("ACENTERAUTONOMOUSMOVEMENT/CENTER-CORAL4-CORALSTATIONTOREEF");

        return routineFollower(routine, firstCycleBargeToReef, secondCycleReefToCoralStation, secondCycleCoralStationToReef,
                thirdCycleReefToCoralStation, thirdCycleCoralStationToReef, fourthCycleReefToCoralStation, fourthCycleCoralStationToReef);
    }

    public Command centerAutoCMD() {
        return centerCompleteAuto().cmd();
    }

    // Left auto
    public AutoRoutine leftCompleteAuto() {
        AutoRoutine routine = follower.factory.newRoutine("Left Auto: Complete");
        AutoTrajectory firstCycleBargeToReef = routine.trajectory("BLEFTSIDEMOVEMENTAUTONOMOUS/LEFTAUTO-CORAL1-BARGETOREEF");
        AutoTrajectory secondCycleReefToCoralStation = routine.trajectory("BLEFTSIDEMOVEMENTAUTONOMOUS/LEFTAUTO-CORAL2-BREEFTOCORALSTATION");
        AutoTrajectory secondCycleCoralStationToReef = routine.trajectory("BLEFTSIDEMOVEMENTAUTONOMOUS/LEFTAUTO-CORAL2-CORALSTATIONTOREEF");
        AutoTrajectory thirdCycleReefToCoralStation = routine.trajectory("BLEFTSIDEMOVEMENTAUTONOMOUS/LEFTAUTO-CORAL3-BREEFTOCORALSTATION");
        AutoTrajectory thirdCycleCoralStationToReef = routine.trajectory("BLEFTSIDEMOVEMENTAUTONOMOUS/LEFTAUTO-CORAL3-CORALSTATIONTOREEF");
        AutoTrajectory fourthCycleReefToCoralStation = routine.trajectory("BLEFTSIDEMOVEMENTAUTONOMOUS/LEFTAUTO-CORAL4-BREEFTOCORALSTATION");
        AutoTrajectory fourthCycleCoralStationToReef = routine.trajectory("BLEFTSIDEMOVEMENTAUTONOMOUS/LEFTAUTO-CORAL4-CORALSTATIONTOREEF");

        return routineFollower(routine, firstCycleBargeToReef, secondCycleReefToCoralStation, secondCycleCoralStationToReef,
                thirdCycleReefToCoralStation, thirdCycleCoralStationToReef, fourthCycleReefToCoralStation, fourthCycleCoralStationToReef);
    }

    public Command leftAutoCMD() {
        return leftCompleteAuto().cmd();
    }

    // Right auto
    public AutoRoutine rightCompleteAuto() {
        AutoRoutine routine = follower.factory.newRoutine("Right Auto: Complete");
        AutoTrajectory firstCycleBargeToReef = routine.trajectory("CRIGHTSIDEMOVEMENTAUTONOMOUS/RIGHTAUTO-CORAL1-BARGETOREEF");
        AutoTrajectory secondCycleReefToCoralStation = routine.trajectory("CRIGHTSIDEMOVEMENTAUTONOMOUS/RIGHTAUTO-CORAL2-BREEFTOCORALSTATION");
        AutoTrajectory secondCycleCoralStationToReef = routine.trajectory("CRIGHTSIDEMOVEMENTAUTONOMOUS/RIGHTAUTO-CORAL2-CORALSTATIONTOREEF");
        AutoTrajectory thirdCycleReefToCoralStation = routine.trajectory("CRIGHTSIDEMOVEMENTAUTONOMOUS/RIGHTAUTO-CORAL3-BREEFTOCORALSTATION");
        AutoTrajectory thirdCycleCoralStationToReef = routine.trajectory("CRIGHTSIDEMOVEMENTAUTONOMOUS/RIGHTAUTO-CORAL3-CORALSTATIONTOREEF");
        AutoTrajectory fourthCycleReefToCoralStation = routine.trajectory("CRIGHTSIDEMOVEMENTAUTONOMOUS/RIGHTAUTO-CORAL4-BREEFTOCORALSTATION");
        AutoTrajectory fourthCycleCoralStationToReef = routine.trajectory("CRIGHTSIDEMOVEMENTAUTONOMOUS/RIGHTAUTO-CORAL4-CORALSTATIONTOREEF");

        return routineFollower(routine, firstCycleBargeToReef, secondCycleReefToCoralStation, secondCycleCoralStationToReef,
                thirdCycleReefToCoralStation, thirdCycleCoralStationToReef, fourthCycleReefToCoralStation, fourthCycleCoralStationToReef);
    }

    public Command autoTest() {
        return Commands.sequence(
                follower.factory.resetOdometry(""),
                follower.factory.trajectoryCmd(""),
                armSystem.setPoseCommand(ArmPoses.L4.getPose(), ArmOrders.JEW.getOrder()),
                Commands.sequence(
                        // TODO: LÓGICA DE LIMELIGHTS PARA ALINEARSE CON REEF
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand() // step 1: leaving piece from cycle 1 (precharged)
                        )
                )
        );
    }


    public Command rightAutoCMD() {
        return rightCompleteAuto().cmd();
    }
}