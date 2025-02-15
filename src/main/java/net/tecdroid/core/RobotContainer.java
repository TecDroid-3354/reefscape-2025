package net.tecdroid.core;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;
import net.tecdroid.subsystems.drivetrain.SwerveDriveConstants;

public class RobotContainer {
    SwerveDrive       swerveDrive = new SwerveDrive(SwerveDriveConstants.CONFIG);

    private final CommandXboxController controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public void setup() {
        swerveDrive.matchModuleSteeringEncodersToAbsoluteEncoders();
    }

}
