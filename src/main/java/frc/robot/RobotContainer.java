// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Swerve.SwerveDriveDriver;
import frc.robot.subsystems.Swerve.SwerveDrive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveDrive swerveDrive = new SwerveDrive();
  SwerveDriveDriver swerveDriver;
  
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    
    swerveDriver = new SwerveDriveDriver(
      () -> new Pair<Double, Double>(-controller.getLeftX(), -controller.getLeftY()),
      () -> new Pair<Double, Double>(-controller.getRightX(), -controller.getRightY())
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
    controller.x().onTrue(Commands.runOnce(swerveDriver::toggleOrientation));
    controller.start().onTrue(Commands.runOnce(swerveDrive::zeroHeading, swerveDrive));
    swerveDrive.setDefaultCommand(Commands.print("a").andThen(Commands.run(() -> {
      swerveDriver.apply(swerveDrive);
    }, swerveDrive)));
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
