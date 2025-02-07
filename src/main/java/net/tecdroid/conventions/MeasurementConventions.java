package net.tecdroid.conventions;

import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public class MeasurementConventions {
    public static final DistanceUnit           MLDU = Meters;
    public static final LinearVelocityUnit     MLVU = MetersPerSecond;
    public static final LinearAccelerationUnit MLAU = MetersPerSecondPerSecond;

    public static final AngleUnit               MADU = Degrees;
    public static final AngularVelocityUnit     MAVU = DegreesPerSecond;
    public static final AngularAccelerationUnit MAAU = DegreesPerSecondPerSecond;

    public static final AngleUnit           OPERAND_ANGLE_UNIT            = Radians;
    public static final AngularVelocityUnit OPERAND_ANGULAR_VELOCITY_UNIT = RadiansPerSecond;
}
