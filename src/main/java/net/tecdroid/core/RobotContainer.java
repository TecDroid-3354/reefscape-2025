package net.tecdroid.core;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import net.tecdroid.constants.GenericConstants;
import net.tecdroid.input.CompliantXboxController;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;
import net.tecdroid.subsystems.drivetrain.SwerveDriveConfiguration;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private final CompliantXboxController controller  =
        new CompliantXboxController(GenericConstants.INSTANCE.getDriverControllerId());
    private final SwerveDrive             swerveDrive =
        new SwerveDrive(SwerveDriveConfiguration.INSTANCE.getConfiguration());

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerveDrive.matchModuleSteeringEncodersToAbsoluteEncoders();

        swerveDrive.setDefaultCommand(Commands.run(() -> {
            ChassisSpeeds speeds = new ChassisSpeeds(MetersPerSecond.of(controller.getLeftY() * 3.0),
                                                     MetersPerSecond.of(controller.getLeftX() * 3.0),
                                                     DegreesPerSecond.of(controller.getRightX() * 720));

            swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, new Rotation2d(swerveDrive.getHeading())));
        }, swerveDrive));

        controller.x().onTrue(Commands.runOnce(() -> {
            swerveDrive.setHeading(Radians.of(0.0));
        }));
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public void setup() {
    }
}
