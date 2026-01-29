package com.hfrobots.tnt.season2526;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.RequiredArgsConstructor;


@RequiredArgsConstructor
public class TimeTracker {
    private long maxTimeMillis = Long.MIN_VALUE;

    private long minTimeMillis = Long.MAX_VALUE;

    private long accumulatedTimeMillis;

    private long timerCounts;

    private final Telemetry telemetry;

    private final String name;

    public void trackTime(Runnable code) {
        long startTimeMillis = System.currentTimeMillis();
        code.run();
        long elapsedTimeMillis = System.currentTimeMillis() - startTimeMillis;
        accumulatedTimeMillis += elapsedTimeMillis;
        timerCounts++;

        if (elapsedTimeMillis < minTimeMillis) {
            minTimeMillis = elapsedTimeMillis;
        }

        if (elapsedTimeMillis > maxTimeMillis) {
            maxTimeMillis = elapsedTimeMillis;
        }

        double averageTimeMillis = (double)accumulatedTimeMillis / (double)timerCounts;

        telemetry.addData("T(" + name + ")", "%d / %d / %d", minTimeMillis, (long)averageTimeMillis, maxTimeMillis);
    }
}
