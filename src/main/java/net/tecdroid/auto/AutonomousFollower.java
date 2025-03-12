package net.tecdroid.auto;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
                true,
                this.swerveDrive
        );

        thetaPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = swerveDrive.getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + xPIDController.calculate(pose.getX(), sample.x),
                sample.vy + yPIDController.calculate(pose.getY(), sample.y),
                sample.omega + thetaPIDController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        swerveDrive.drive(speeds);
    }
}