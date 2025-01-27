package net.tecdroid.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class UnitConstants {
    public static final Angle FULL_ROTATION    = Units.Degrees.of(360);
    public static final Angle HALF_ROTATION    = FULL_ROTATION.div(2);
    public static final Angle QUARTER_ROTATION = HALF_ROTATION.div(2);
}
