// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDrive;

import java.util.function.Supplier;

import static frc.robot.util.SwerveDriveUtil.denormalizeLinearVelocity;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveDriveCommand extends Command {

  private final SwerveDrive swerveDriveSubsystem;
  private Supplier<Double> forwardPowerSource, strafePowerSource;
  private Supplier<Rotation2d> rotationTargetSource;
  private Supplier<Boolean> fodTrigger;
  private Supplier<Boolean> resetGyroSupplier;

  private Rotation2d lastAngle = Rotation2d.fromDegrees(0);

  boolean fod = true;
  boolean fodFlag = false;

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(SwerveDrive swerveDriveSubsytem,
    Supplier<Double> forwardPower, Supplier<Double> strafePower, Supplier<Rotation2d> rotationTarget, Supplier<Boolean> isFod, Supplier<Boolean> resetSupplier) {
    
    this.swerveDriveSubsystem = swerveDriveSubsytem;
    this.forwardPowerSource = forwardPower;
    this.strafePowerSource = strafePower;
    this.rotationTargetSource = rotationTarget;
    this.resetGyroSupplier = resetSupplier;

    this.fodTrigger = isFod;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsytem);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (fodTrigger.get() && !fodFlag) {
      fod = !fod;
      fodFlag = true;
    }

    if (fodTrigger.get() && fodFlag) {
      fodFlag = false;
    }

    if (resetGyroSupplier.get()) {
      swerveDriveSubsystem.zeroHeading();
    }

    final double xVel = denormalizeLinearVelocity(forwardPowerSource.get());
    final double yVel = denormalizeLinearVelocity(strafePowerSource.get());

    SmartDashboard.putNumber("yvel", yVel);
    SmartDashboard.putNumber("xvel", xVel);
    
    final Rotation2d wTarget = rotationTargetSource.get().getDegrees() <= 360.0 ? rotationTargetSource.get() : lastAngle;
    lastAngle = wTarget;

    ShuffleboardTab tab = Shuffleboard.getTab("GyroTab");
    tab.addDouble("TargetAngle", () -> wTarget.getDegrees());

    final ChassisSpeeds speeds = fod ? ChassisSpeeds.fromRobotRelativeSpeeds(xVel, yVel, 0.0, swerveDriveSubsystem.getHeading())
                                     : new ChassisSpeeds(xVel, yVel, 0.0);

    swerveDriveSubsystem.drive(speeds, wTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
