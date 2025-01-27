package net.tecdroid.conventions;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;

import static edu.wpi.first.units.Units.*;

public class MeasurementConventions {
    public static final DistanceUnit       MEASUREMENT_DISTANCE_UNIT        = Meters;
    public static final LinearVelocityUnit MEASUREMENT_LINEAR_VELOCITY_UNIT = MetersPerSecond;

    public static final AngleUnit           MEASUREMENT_ANGLE_UNIT            = Degrees;
    public static final AngularVelocityUnit MEASUREMENT_ANGULAR_VELOCITY_UNIT = DegreesPerSecond;

    public static final AngleUnit           OPERAND_ANGLE_UNIT            = Radians;
    public static final AngularVelocityUnit OPERAND_ANGULAR_VELOCITY_UNIT = RadiansPerSecond;
}
