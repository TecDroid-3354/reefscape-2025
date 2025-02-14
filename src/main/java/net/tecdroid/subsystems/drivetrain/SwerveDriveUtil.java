package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;
import static net.tecdroid.constants.Constants.SwerveConstants.Modules.PhysicsConstants.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveConstants.*;

public class SwerveDriveUtil {
    public static LinearVelocity denormalizeLinearVelocity(double normalizedSpeeds) {
        return DRIVE_MAX_LINEAR_VELOCITY.times(normalizedSpeeds);
    }

    public static AngularVelocity convertFromDriveLinearVelocityToDriveAngularVelocity(LinearVelocity linearVelocity) {
        return RotationsPerSecond.of(DRIVE_GR.unapply(linearVelocity.in(MetersPerSecond) / WHEEL_CIRCUMFERENCE.in(Meters)));
    }

    public static LinearVelocity convertFromWheelAngularVelocityToWheelLinearVelocity(AngularVelocity angularVelocity) {
        return WHEEL_CIRCUMFERENCE.times(angularVelocity.in(RotationsPerSecond)).per(Second);
    }

    public static double denormalizeAngularVelocity(double normalizedSpeeds) {
        return normalizedSpeeds * MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND;
    }

    public static AngularVelocity convertFromWheelLinearVelocityToDriveAngularVelocity(LinearVelocity linearVelocity) {
        return RotationsPerSecond.of(DRIVE_GR.unapply(linearVelocity.in(MetersPerSecond) / WHEEL_CIRCUMFERENCE.in(Meters)));
    }

//    public static AngularVelocity driveVelocityToAngularVelocity(LinearVelocity velocity) {
//        final double velocityMs = (velocity.in(MetersPerSecond) / WHEEL_DIAMETER.in(Meters));
//        return DRIVE_VCF.div(velocityMs);
//    }

}
