package net.tecdroid.core;

import edu.wpi.first.wpilibj2.command.Command;
import net.tecdroid.constants.GenericConstants;
import net.tecdroid.input.Controller;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;
import net.tecdroid.subsystems.drivetrain.SwerveDriveConstants;

public class RobotContainer {
    private final Controller controller = new Controller(GenericConstants.INSTANCE.getDriverControllerId());

    private final SwerveDrive swerveDrive = new SwerveDrive(SwerveDriveConstants.CONFIG);

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
