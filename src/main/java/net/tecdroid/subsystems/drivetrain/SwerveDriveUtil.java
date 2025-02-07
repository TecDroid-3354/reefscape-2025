package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import javax.sound.sampled.Line;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static net.tecdroid.constants.Constants.SwerveConstants.Modules.PhysicsConstants.MAX_SPEED_METERS_PER_SECOND;
import static net.tecdroid.constants.Constants.SwerveConstants.Modules.PhysicsConstants.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.DRIVE_VCF;

public class SwerveDriveUtil {
    public static double denormalizeLinearVelocity(double normalizedSpeeds) {
        return normalizedSpeeds * MAX_SPEED_METERS_PER_SECOND;
        
    }

    public static double denormalizeAngularVelocity(double normalizedSpeeds) {
        return normalizedSpeeds * MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND;
    }

    public static AngularVelocity driveVelocityToAngularVelocity(LinearVelocity velocity) {
        final double velocityMs = velocity.in(MetersPerSecond);
        return DRIVE_VCF.div(velocityMs);
    }

}
