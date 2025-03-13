package net.tecdroid.auto;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;

import static net.tecdroid.auto.AutonomousConstants.*;

public class AutonomousFollower {
    public final AutoFactory factory;
    private final PIDController xPIDController = new PIDController(
            forwardPID.getP(), forwardPID.getI(), forwardPID.getD()
    );
    private final PIDController yPIDController = new PIDController(
            sidewaysPID.getP(), sidewaysPID.getI(), sidewaysPID.getD()
    );
    private final PIDController thetaPIDController = new PIDController(
            rotationalPID.getP(), rotationalPID.getI(), rotationalPID.getD()
    );

    private final SwerveDrive swerveDrive;

    public AutonomousFollower(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        factory = new AutoFactory(
                this.swerveDrive::getPose,
                this.swerveDrive::resetOdometry,
                this::followTrajectory,
                DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red),
                this.swerveDrive
        );

        thetaPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = swerveDrive.getPose();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                sample.vx + xPIDController.calculate(pose.getX(), sample.x) * 0.6,
                sample.vy + yPIDController.calculate(pose.getY(), sample.y) * 0.6,
                sample.omega + thetaPIDController.calculate(pose.getRotation().getRadians(), sample.heading) * 0.6,
                new Rotation2d(swerveDrive.getHeading())
        );

        swerveDrive.drive(speeds);
    }
}