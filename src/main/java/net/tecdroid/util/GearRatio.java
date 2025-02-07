package net.tecdroid.util;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor @Getter
public class GearRatio {
    private final double numerator;
    private final double denominator;
    private final double ratio = numerator / denominator;

    public double apply(double value) {
        return value / ratio;
    }

    public double unapply(double value) {
        return value * ratio;
    }
}
