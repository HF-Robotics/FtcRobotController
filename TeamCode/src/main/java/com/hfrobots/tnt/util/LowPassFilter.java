package com.hfrobots.tnt.util;

public class LowPassFilter {

    private final double filterFactor;

    private double oldValue = Float.MIN_VALUE;

    private double maxDeltaPosition = Float.MIN_VALUE;

    public LowPassFilter(final double filterFactor) {
        this.filterFactor = filterFactor;
    }

    public double filter(final double input) {

        if (oldValue == Float.MIN_VALUE) {
            oldValue = input;
            return input;
        }

        double filtered = (input - oldValue) * filterFactor + oldValue;

        oldValue = filtered;

        return filtered;
    }
}