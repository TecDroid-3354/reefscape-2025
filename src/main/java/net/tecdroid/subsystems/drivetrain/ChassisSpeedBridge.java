package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.List;

public class ChassisSpeedBridge {
    static ChassisSpeeds ktToJavaArrayChassisSpeeds(SwerveDriveKinematics kinematics, List<SwerveModuleState> array) {
        return kinematics.toChassisSpeeds(array.get(0), array.get(1), array.get(2), array.get(3));
    }
}
