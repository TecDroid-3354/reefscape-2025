package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.subsystems.DriveTrain.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, xTurningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    private boolean fieldOrientedActived = false;
    // Chasis speeds
    public ChassisSpeeds chassisSpeeds;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> xTurningSpdFunction, Supplier<Boolean> fieldOrientedFunction) {

        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.xTurningSpdFunction = xTurningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);

        //Shuffle board tab
        this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve Tab");
        swerveTab.addDouble("X: : ", () -> {return getXSpeed(this.chassisSpeeds);});
        swerveTab.addDouble("Y: ", () -> {return getYSpeed(this.chassisSpeeds);});
        swerveTab.addDouble("Omega: ", () -> {return getOmegaSpeed(this.chassisSpeeds);});
    }

    @Override
    public void initialize() {
    }

    public void changeFieldOriented(boolean fieldOrientedBtn) {
        if (fieldOrientedBtn) {
            fieldOrientedActived = !fieldOrientedActived;
            swerveSubsystem.zeroHeading();
        }
    }

    @Override
    public void execute() {
        // Change the drive mode
        changeFieldOriented(fieldOrientedFunction.get());

        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = xTurningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedActived) {
            //  Relative to field (set the gyro inverted)
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            
            // Shuffleboard tab chassis speeds
            this.chassisSpeeds = chassisSpeeds;
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

            // Shuffleboard tab chassis speeds
            this.chassisSpeeds = chassisSpeeds;
        }

        // 5. Output chassis speeds
        swerveSubsystem.setChassisSpeed(chassisSpeeds);
    }

    public double getXSpeed(ChassisSpeeds chassisSpeeds) {
        return chassisSpeeds.vxMetersPerSecond;
    }

    public double getYSpeed(ChassisSpeeds chassisSpeeds) {
        return chassisSpeeds.vyMetersPerSecond;
    }

    public double getOmegaSpeed(ChassisSpeeds chassisSpeeds) {
        return chassisSpeeds.omegaRadiansPerSecond;
    }


    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}