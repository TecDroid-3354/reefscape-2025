// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.tecdroid.core;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import net.tecdroid.constants.Constants.OperatorConstants;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;
import net.tecdroid.subsystems.drivetrain.SwerveDriveConstants;
import net.tecdroid.subsystems.drivetrain.SwerveDriveDriver;
import net.tecdroid.subsystems.drivetrain.SwerveTuner;
import org.joml.Vector2d;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveUtil.convertFromWheelLinearVelocityToDriveAngularVelocity;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    SwerveDrive       swerveDrive = new SwerveDrive(SwerveDriveConstants.CONFIG);
    SwerveDriveDriver swerveDriver;
    SwerveTuner tuner = new SwerveTuner(swerveDrive);

    private final CommandXboxController controller =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        swerveDriver = new SwerveDriveDriver(
                () -> new Vector2d(controller.getLeftX(), controller.getLeftY()),
                () -> new Vector2d(controller.getRightX(), controller.getRightY()).mul(0)
        );

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        swerveDrive.setDefaultCommand(Commands.run(() -> { swerveDriver.apply(swerveDrive); }, swerveDrive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }

    public void setup() {
        swerveDrive.seedEncoders();
    }

}
