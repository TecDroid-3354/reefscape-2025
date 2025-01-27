package net.tecdroid.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

import static java.lang.Math.sqrt;

@SuppressWarnings("unchecked")
public class UnitHelpers {
    public static <M extends Measure<? extends Unit>> M hypot(M x, M y) {
        final double xv = x.in(x.baseUnit());
        final double yv = y.in(y.baseUnit());
        final double h  = sqrt(xv * xv + yv * yv);
        return (M) x.baseUnit()
                    .of(h);
    }
}
