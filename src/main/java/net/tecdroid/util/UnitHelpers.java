package net.tecdroid.util;

import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;
import static java.lang.Math.sqrt;

public class UnitHelpers {
    public static Distance hypot(Distance x, Distance y) {
        final double xv = x.in(Meters);
        final double yv = y.in(Meters);
        final double h  = sqrt(xv * xv + yv * yv);
        return Meters.of(h);
    }
}
