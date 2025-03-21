package net.tecdroid.vision.limelight.systems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import net.tecdroid.constants.StringConstantsKt;
import net.tecdroid.subsystems.drivetrain.SwerveDrive;
import net.tecdroid.util.LimeLightChoice;
import net.tecdroid.vision.limelight.Limelight;
import net.tecdroid.vision.limelight.LimelightConfig;

public class LimelightPoseEstimator {
    private final SwerveDrive drive;
    private final Limelight leftLimelight = new Limelight(new LimelightConfig(StringConstantsKt.leftLimelightName, new Translation3d()));
    private final Limelight rightLimelight = new Limelight(new LimelightConfig(StringConstantsKt.rightLimelightName, new Translation3d()));

    boolean isRedAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

    public LimelightPoseEstimator(SwerveDrive drive) {
        this.drive = drive;

    }

    public void updateOdometry(LimeLightChoice choice) {
        Limelight limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;

        boolean doRejectUpdate = false;

        limelight.setRobotOrientation(drive.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees());
        LimelightHelpers.PoseEstimate mt2 = isRedAlliance ? limelight.getBotPoseEstimate_wpiRed_MegaTag2() :  limelight.getBotPoseEstimate_wpiBlue_MegaTag2();

        if(Math.abs(drive.getImu().getAngularVelocityZWorld().getValueAsDouble()) > 720) {
                doRejectUpdate = true;
        }

        if(mt2.tagCount == 0) {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate) {
            drive.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            drive.getPoseEstimator().addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
        }

    }
}
