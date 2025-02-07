package net.tecdroid.util;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class GearRatio {
    private final double numerator;
    private final double denominator;
    private final double ratio;

    public GearRatio(final double numerator, final double denominator) {
        this.numerator = numerator;
        this.denominator = denominator;
        this.ratio = numerator / denominator;
    }

    public double apply(double value) {
        return value / ratio;
    }

    public double unapply(double value) {
        return value * ratio;
    }

    public double getNumerator() {
        return numerator;
    }

    public double getDenominator() {
        return denominator;
    }

    public double getRatio() {
        return ratio;
    }
}
