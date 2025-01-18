package frc.robot.util;

import static frc.robot.Constants.SwerveConstants.Modules.PhysicsConstants.MAX_SPEED_METERS_PER_SECOND;
import static frc.robot.Constants.SwerveConstants.Modules.PhysicsConstants.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND;

public class SwerveDriveUtil {
    public static double denormalizeLinearVelocity(double normalizedSpeeds) {
        return normalizedSpeeds * MAX_SPEED_METERS_PER_SECOND;
        
    }

    public static double denormalizeAngularVelocity(double normalizedSpeeds) {
        return normalizedSpeeds * MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND;
    }

}
