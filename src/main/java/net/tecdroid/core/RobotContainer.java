package net.tecdroid.core;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import net.tecdroid.Auto.AutoRoutines;
import net.tecdroid.constants.GenericConstants;
import net.tecdroid.input.Controller;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;
import net.tecdroid.subsystems.drivetrain.SwerveDriveConstants;
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver;
import net.tecdroid.subsystems.drivetrain.SwerveModule;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private final Controller controller = new Controller(GenericConstants.INSTANCE.getDriverControllerId());

    Supplier<LinearVelocity> vx = () -> MetersPerSecond.of(controller.getLeftY() * 3.0);
    Supplier<LinearVelocity> vy = () -> MetersPerSecond.of(controller.getLeftX() * 3.0);
    Supplier<AngularVelocity> vw = () -> DegreesPerSecond.of(controller.getRightX() * 700.0);

    private final SwerveDrive swerveDrive = new SwerveDrive(SwerveDriveConstants.INSTANCE.getSwerveDriveConfig());
    private final SwerveDriveDriver driveDriver = new SwerveDriveDriver();
    private final AutoRoutines autoRoutines = new AutoRoutines(swerveDrive);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        driveDriver.setForwardsVelocitySupplier(vx);
        driveDriver.setSidewaysVelocitySupplier(vy);
        driveDriver.setAngularVelocitySupplier(vw);

        swerveDrive.setDefaultCommand(Commands.run(() -> {
            var speeds = driveDriver.obtainTargetSpeeds(new Rotation2d(swerveDrive.getHeading()));
            swerveDrive.drive(speeds);
        }, swerveDrive));

        controller.rightBumper().onTrue(autoRoutines.runTwoMeterCMD());
        controller.leftBumper().onTrue(autoRoutines.runMinusTwoMeterCMD());

        SmartDashboard.putString("FRP", SwerveDriveConstants.ModuleState.INSTANCE.getFrontRightModuleOffset().toString());
        SmartDashboard.putString("FLP", SwerveDriveConstants.ModuleState.INSTANCE.getFrontLeftModuleOffset().toString());
        SmartDashboard.putString("BLP", SwerveDriveConstants.ModuleState.INSTANCE.getBackLeftModuleOffset().toString());
        SmartDashboard.putString("BRP", SwerveDriveConstants.ModuleState.INSTANCE.getBackRightModuleOffset().toString());

    }

    public Command getAutonomousCommand() {
        return null;
    }

    public void setup() {
        swerveDrive.matchModuleSteeringEncodersToAbsoluteEncoders();
    }

}
