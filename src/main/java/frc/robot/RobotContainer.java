// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.DriveTrain.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  private final XboxController controller = new XboxController(0);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> controller.getRightStickButtonPressed()));

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(controller, XboxController.Button.kStart.value).onTrue(swerveSubsystem.runOnce(swerveSubsystem::zeroHeading));

  }

  public Command getAutonomousCommand() {

    return new PathPlannerAuto("ExamplePath");
  }
}
