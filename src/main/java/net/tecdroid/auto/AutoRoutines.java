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


    public AutoRoutine leftHardCodedCompleteAuto() {
        AutoRoutine routine = follower.factory.newRoutine("Left Auto: Complete");
        AutoTrajectory firstCycleBargeToReef = routine.trajectory("left-cycle1-bargeToReef");
        AutoTrajectory secondCycleReefToCoralStation = routine.trajectory("left-cycle2-reefToCoralStation");
        AutoTrajectory secondCycleCoralStationToReef = routine.trajectory("left-cycle2-coralStationToReef");
        AutoTrajectory thirdCycleReefToCoralStation = routine.trajectory("left-cycle3-reefToCoralStation");
        AutoTrajectory thirdCycleCoralStationToReef = routine.trajectory("left-cycle3-coralStationToReef");

        // Executes when routine starts
        routine.active().onTrue(
                Commands.sequence(
                        firstCycleBargeToReef.resetOdometry(),
                        Commands.parallel(
                                firstCycleBargeToReef.cmd(),
                                armSystem.setPoseCommand(ArmPoses.L4.getPose(), ArmOrders.JEW.getOrder())
                        )
                        // TODO: Limelight logic
                        //limelights.alignYAxisToAprilTagDetection(swerveDriver, )
                )
        );


        // ####################
        // # GENERAL COMMANDS #
        // ####################

        // ! Cycle #2 : Second coral

        //firstCycleBargeToReef.done().onTrue(intake.setVoltageCommand(Volts.of(10.0))); // ✅
        firstCycleBargeToReef.done().onTrue(
                Commands.sequence(
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand(), // step 1: leaving piece from cycle 1 (precharged)
                                Commands.parallel(
                                        secondCycleReefToCoralStation.cmd(),
                                        armSystem.setPoseCommand(ArmPoses.CoralStation.getPose(), ArmOrders.EJW.getOrder())
                                )
                        )
                )
        );

        secondCycleReefToCoralStation.done().onTrue(
                Commands.sequence(
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(intake::hasCoral).andThen(
                                Commands.sequence(
                                        intake.stopCommand(), // step 3: intaking at coral station
                                        Commands.parallel(
                                                secondCycleCoralStationToReef.cmd(), // step 4: going from coral station to reef
                                                armSystem.setPoseCommand(ArmPoses.L4.getPose(), ArmOrders.JEW.getOrder())
                                        )

                                )
                        )
                )
        );

        secondCycleCoralStationToReef.done().onTrue(
                Commands.sequence(
                        // TODO: AQUÍ PONER LÓGICA DE LIMELIGHTS PARA ALINEARSE CON EL REEF
                        // limelights.alignInAllAxis(swerveDriver, Degrees.of(0.0), Inches.of(5.0), true),
                        /*Commands.waitUntil(() -> limelights.isAlignedAtReef(true) || Commands.waitTime(Seconds.of(3))
                                .isFinished()).andThen(intake.setVoltageCommand(Volts.of(10.0))),*/
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand(),
                                Commands.parallel(
                                        thirdCycleReefToCoralStation.cmd(),
                                        armSystem.setPoseCommand(ArmPoses.CoralStation.getPose(), ArmOrders.EJW.getOrder())
                                )
                        )
                )
        );

        thirdCycleReefToCoralStation.done().onTrue(
                Commands.sequence(
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(intake::hasCoral).andThen(
                                Commands.sequence(
                                        intake.stopCommand(),
                                        Commands.parallel(
                                                thirdCycleCoralStationToReef.cmd(), // step 4: going from coral station to reef
                                                armSystem.setPoseCommand(ArmPoses.L4.getPose(), ArmOrders.JEW.getOrder())
                                        )
                                )
                        )
                )
        );

        thirdCycleCoralStationToReef.done().onTrue(
                Commands.sequence(
                        // TODO: AQUÍ PONER LÓGICA DE LIMELIGHTS PARA ALINEARSE CON EL REEF
                        // limelights.alignInAllAxis(swerveDriver, Degrees.of(0.0), Inches.of(5.0), true),
                        /*Commands.waitUntil(() -> limelights.isAlignedAtReef(true) || Commands.waitTime(Seconds.of(3))
                                .isFinished()).andThen(intake.setVoltageCommand(Volts.of(10.0))),*/
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand()
                        )
                )
        );

        return routine;
    }

    public Command leftHardCodedAutoCMD() {
        return leftHardCodedCompleteAuto().cmd();
    }

    public AutoRoutine centerHardCodedCompleteAuto() {
        AutoRoutine routine = follower.factory.newRoutine("Center Auto: Complete");
        AutoTrajectory firstCycleBargeToReef = routine.trajectory("center-cycle1-bargeToReef");
        AutoTrajectory secondCycleReefToCoralStation = routine.trajectory("center-cycle2-reefToCoralStation");
        AutoTrajectory secondCycleCoralStationToReef = routine.trajectory("center-cycle2-coralStationToReef");
        AutoTrajectory thirdCycleReefToCoralStation = routine.trajectory("center-cycle3-reefToCoralStation");
        AutoTrajectory thirdCycleCoralStationToReef = routine.trajectory("center-cycle3-coralStationToReef");

        // Executes when routine starts
        routine.active().onTrue(
                Commands.sequence(
                        firstCycleBargeToReef.resetOdometry(),
                        Commands.parallel(
                                firstCycleBargeToReef.cmd(),
                                armSystem.setPoseCommand(ArmPoses.L4.getPose(), ArmOrders.JEW.getOrder())
                        )
                        // TODO: Limelight logic
                        //limelights.alignYAxisToAprilTagDetection(swerveDriver, )
                )
        );


        // ####################
        // # GENERAL COMMANDS #
        // ####################

        // ! Cycle #2 : Second coral

        //firstCycleBargeToReef.done().onTrue(intake.setVoltageCommand(Volts.of(10.0))); // ✅
        firstCycleBargeToReef.done().onTrue(
                Commands.sequence(
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand(), // step 1: leaving piece from cycle 1 (precharged)
                                Commands.parallel(
                                        secondCycleReefToCoralStation.cmd(),
                                        armSystem.setPoseCommand(ArmPoses.CoralStation.getPose(), ArmOrders.EJW.getOrder())
                                )
                        )
                )
        );

        secondCycleReefToCoralStation.done().onTrue(
                Commands.sequence(
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(intake::hasCoral).andThen(
                                Commands.sequence(
                                        intake.stopCommand(), // step 3: intaking at coral station
                                        Commands.parallel(
                                                secondCycleCoralStationToReef.cmd(), // step 4: going from coral station to reef
                                                armSystem.setPoseCommand(ArmPoses.L4.getPose(), ArmOrders.JEW.getOrder())
                                        )

                                )
                        )
                )
        );

        secondCycleCoralStationToReef.done().onTrue(
                Commands.sequence(
                        // TODO: AQUÍ PONER LÓGICA DE LIMELIGHTS PARA ALINEARSE CON EL REEF
                        // limelights.alignInAllAxis(swerveDriver, Degrees.of(0.0), Inches.of(5.0), true),
                        /*Commands.waitUntil(() -> limelights.isAlignedAtReef(true) || Commands.waitTime(Seconds.of(3))
                                .isFinished()).andThen(intake.setVoltageCommand(Volts.of(10.0))),*/
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand(),
                                Commands.parallel(
                                        thirdCycleReefToCoralStation.cmd(),
                                        armSystem.setPoseCommand(ArmPoses.CoralStation.getPose(), ArmOrders.EJW.getOrder())
                                )
                        )
                )
        );

        thirdCycleReefToCoralStation.done().onTrue(
                Commands.sequence(
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(intake::hasCoral).andThen(
                                Commands.sequence(
                                        intake.stopCommand(),
                                        Commands.parallel(
                                                thirdCycleCoralStationToReef.cmd(), // step 4: going from coral station to reef
                                                armSystem.setPoseCommand(ArmPoses.L4.getPose(), ArmOrders.JEW.getOrder())
                                        )
                                )
                        )
                )
        );

        thirdCycleCoralStationToReef.done().onTrue(
                Commands.sequence(
                        // TODO: AQUÍ PONER LÓGICA DE LIMELIGHTS PARA ALINEARSE CON EL REEF
                        // limelights.alignInAllAxis(swerveDriver, Degrees.of(0.0), Inches.of(5.0), true),
                        /*Commands.waitUntil(() -> limelights.isAlignedAtReef(true) || Commands.waitTime(Seconds.of(3))
                                .isFinished()).andThen(intake.setVoltageCommand(Volts.of(10.0))),*/
                        intake.setVoltageCommand(Volts.of(10.0)),
                        Commands.waitUntil(() -> !intake.hasCoral() || Commands.waitTime(Seconds.of(2))
                                .isFinished()).andThen(
                                intake.stopCommand()
                        )
                )
        );

        return routine;
    }

    public Command centerHardCodedAutoCMD() {
        return centerHardCodedCompleteAuto().cmd();
    }
}