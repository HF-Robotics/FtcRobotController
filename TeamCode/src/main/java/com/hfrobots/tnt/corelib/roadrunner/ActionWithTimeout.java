package com.hfrobots.tnt.corelib.roadrunner;

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.google.common.base.Stopwatch;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;

/**
 * Base functionality to keep track of initialization and
 * a timer to exit if the action takes too long (useful for
 * mechanisms that have state machines of their own).
 */
public abstract class ActionWithTimeout implements Action {
    private Stopwatch stopwatch = Stopwatch.createUnstarted();

    private final long timeoutInMillis;

    private boolean initialized = false;

    protected ActionWithTimeout(final long timeoutInMillis) {
        this.timeoutInMillis = timeoutInMillis;
    }

    protected abstract void initialize();

    protected abstract boolean runUnderTimeout();

    protected abstract void onTimeout();

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            stopwatch.start();
            initialize();
            initialized = true;
        }

        if (stopwatch.elapsed(TimeUnit.MILLISECONDS) > timeoutInMillis) {
            Log.e(LOG_TAG, "Action " + getClass().getName() + " timed out");

            onTimeout();

            return false;
        }

        return runUnderTimeout();
    }
}
