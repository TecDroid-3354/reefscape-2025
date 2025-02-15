package net.tecdroid.Auto;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;

import static net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.AutoConstants.INSTANCE;

public class AutonomousFollower {
    public final AutoFactory factory;
    private final PIDController xPIDController = new PIDController(
            INSTANCE.getForwardPidf().getP(), INSTANCE.getForwardPidf().getI(), INSTANCE.getForwardPidf().getD()
    );
    private final PIDController yPIDController = new PIDController(
            INSTANCE.getSidewaysPidf().getP(), INSTANCE.getSidewaysPidf().getI(), INSTANCE.getSidewaysPidf().getD()
    );
    private final PIDController thetaPIDController = new PIDController(
            INSTANCE.getRotationPidf().getP(), INSTANCE.getRotationPidf().getI(), INSTANCE.getRotationPidf().getD()
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
                sample.vy + xPIDController.calculate(pose.getY(), sample.y),
                sample.omega + thetaPIDController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        swerveDrive.drive(speeds);
    }
}
