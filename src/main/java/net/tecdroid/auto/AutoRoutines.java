package net.tecdroid.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import net.tecdroid.subsystems.intake.*;
import net.tecdroid.subsystems.limeLight.LimeLightsController;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver;
import net.tecdroid.systems.arm.ArmController;
import net.tecdroid.systems.arm.ArmPositions;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveConfigurationKt.getSwerveDriveConfiguration;
import static net.tecdroid.subsystems.intake.IntakeConfigurationKt.getIntakeConfig;

public class AutoRoutines {
    private final AutonomousFollower follower;
    private final Intake intake = new Intake(getIntakeConfig());
    private final DigitalInput intakeSensor = new DigitalInput(4); // digital input --> invertir la señal (cuando detecta algo, retorna false)
    private final LimeLightsController limelights = new LimeLightsController();
    private final SwerveDrive swerveDriveSubsystem = new SwerveDrive(getSwerveDriveConfiguration());
    private final SwerveDriveDriver swerveDriver = new SwerveDriveDriver(
        swerveDriveSubsystem.getMaxLinearVelocity(),
        swerveDriveSubsystem.getMaxAngularVelocity(),
        Seconds.of(0.1),
        Seconds.of(0.1));


    // Arm integration
    private final ArmController arm = new ArmController();
    private final ArmPositions armPositions = new ArmPositions();


    public AutoRoutines() {
        follower = new AutonomousFollower(this.swerveDriveSubsystem);
    }

    /*public AutoRoutine runTwoMeters() {
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
    }*/



    // Individual segments

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

    public Command runTwoMetersCMD() {
        return runTwoMeters().cmd();
    }

    public AutoRoutine leftAutoFirstCycle() {
        AutoRoutine routine = follower.factory.newRoutine("First cycle");
        AutoTrajectory firstCycle = routine.trajectory("LeftAuto-Coral1-Barge-to-Reef");

        routine.active().onTrue(
                Commands.sequence(
                        firstCycle.resetOdometry(),
                        firstCycle.cmd()
                )
        );

        return routine;
    }

    public Command leftAutoFirstCycleCMD() {
        return leftAutoFirstCycle().cmd();
    }






    // Complete auto


    public AutoRoutine leftCompleteAuto() {
        AutoRoutine routine = follower.factory.newRoutine("Left Auto: Complete");
        AutoTrajectory firstCycleBargeToReef = routine.trajectory("LeftAuto-Coral1-Barge-to-Reef");
        AutoTrajectory secondCycleReefToCoralStation = routine.trajectory("LeftAuto-Coral2-Reef-to-Coral-Station");
        AutoTrajectory secondCycleCoralStationToReef = routine.trajectory("LeftAuto-Coral2-Coral-Station-to-Reef");
        AutoTrajectory thirdCycleReefToCoralStation = routine.trajectory("LeftAuto-Coral3-Reef-to-Coral-Station");
        AutoTrajectory thirdCycleCoralStationToReef = routine.trajectory("LeftAuto-Coral3-Coral-Station-to-Reef");

        // Executes when routine starts
        routine.active().onTrue(
                Commands.sequence(
                        firstCycleBargeToReef.resetOdometry(),
                        firstCycleBargeToReef.cmd(),
                        // TODO: Limelight logic
                            //limelights.alignYAxisToAprilTagDetection(swerveDriver, )

                        // ✅ TODO: Arm logic — place arm in L4
                        arm.setArmPoseCMD(armPositions.reefL4),

                        // ✅ TODO: Intake logic — outtake
                        intake.setVoltageCommand(Volts.of(10.0))
                )
        );


        // ####################
        // # GENERAL COMMANDS #
        // ####################

        // All reef -> coralStation cycles will automatically set arm to have coral station intake position
        routine.anyActive(secondCycleReefToCoralStation, thirdCycleReefToCoralStation)
                .whileTrue(arm.setArmPoseCMD(armPositions.coralStationIntake));
                // is equal to:
                // secondCycleReefToCoralStation.active().onTrue(arm.setArmPoseCMD(armPositions.coralStationIntake));
                // thirdCycleReefToCoralStation.active().onTrue(arm.setArmPoseCMD(armPositions.coralStationIntake));

        // All coralStation -> reef cycles will automatically set arm to have reef L4 position
        routine.anyActive(secondCycleCoralStationToReef, thirdCycleCoralStationToReef)
                .whileTrue(arm.setArmPoseCMD(armPositions.reefL4));
                // is equal to:
                // secondCycleCoralStationToReef.active().onTrue(arm.setArmPoseCMD(armPositions.reefL4));
                // thirdCycleCoralStationToReef.active().onTrue(arm.setArmPoseCMD(armPositions.reefL4));




        // ! Cycle #2 : Second coral

        firstCycleBargeToReef.done().onTrue(intake.setVoltageCommand(Volts.of(10.0))); // ✅

        firstCycleBargeToReef.done().and(() -> !intake.hasCoral()).onTrue(secondCycleReefToCoralStation.cmd()); // ✅

        secondCycleReefToCoralStation.done().and(intake::hasCoral).onTrue(secondCycleCoralStationToReef.cmd()); // ✅

        secondCycleCoralStationToReef.done().and(intake::hasCoral).onTrue(intake.setVoltageCommand(Volts.of(10.0))); // ✅

        // TODO: ALIGN WITH LIMELIGHT



        return routine;
    }

    public Command leftAutoRoutineCMD() {
        return leftCompleteAuto().cmd();
    }
}