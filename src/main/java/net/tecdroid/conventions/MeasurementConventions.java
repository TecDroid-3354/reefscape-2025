package net.tecdroid.conventions;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;

import static edu.wpi.first.units.Units.*;

public class MeasurementConventions {
    public static final DistanceUnit       MLDU = Meters;
    public static final LinearVelocityUnit MLVU = MetersPerSecond;

    public static final AngleUnit           MADU = Degrees;
    public static final AngularVelocityUnit MAVU = DegreesPerSecond;

    public static final AngleUnit           OPERAND_ANGLE_UNIT            = Radians;
    public static final AngularVelocityUnit OPERAND_ANGULAR_VELOCITY_UNIT = RadiansPerSecond;
}
