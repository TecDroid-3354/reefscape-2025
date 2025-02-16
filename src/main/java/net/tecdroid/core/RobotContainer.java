package net.tecdroid.core;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import net.tecdroid.constants.GenericConstants;
import net.tecdroid.input.Controller;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;
import net.tecdroid.subsystems.drivetrain.SwerveDriveConstants;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class RobotContainer {
    private final Controller controller = new Controller(GenericConstants.INSTANCE.getDriverControllerId());
    private final SwerveDrive swerveDrive = new SwerveDrive(SwerveDriveConstants.INSTANCE.getSwerveDriveConfig());

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerveDrive.matchModuleSteeringEncodersToAbsoluteEncoders();

        swerveDrive.setDefaultCommand(Commands.run(() -> {
            swerveDrive.drive(
                    new ChassisSpeeds(
                            MetersPerSecond.of(controller.getLeftY() * 3.0),
                            MetersPerSecond.of(controller.getLeftX() * 3.0),
                            DegreesPerSecond.of(controller.getRightX() * 720)
                    )
            );
        }, swerveDrive));
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public void setup() {
        swerveDrive.matchModuleSteeringEncodersToAbsoluteEncoders();
        var tab = Shuffleboard.getTab("Tab");

        int i =  0;
        for (var module : swerveDrive.getModules()) {
            tab.add("Module " + i, module);
            i++;
        }
    }

}
