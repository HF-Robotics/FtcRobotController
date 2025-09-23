package com.hfrobots.tnt.season2526;

import android.util.Log;

import static com.ftc9929.corelib.Constants.LOG_TAG;

import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Ticker;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;

public class PedroFollowerState extends StopwatchTimeoutSafetyState {
    private final Follower follower;

    private final PathChain pathChain;

    private boolean followerHasStarted = false;

    public PedroFollowerState(@NonNull String name, Telemetry telemetry, @NonNull Ticker ticker, final Follower follower, final Path path) {
        this(name, telemetry, ticker, follower, new PathChain(path));
    }

    public PedroFollowerState(@NonNull String name, Telemetry telemetry, @NonNull Ticker ticker, final Follower follower, final PathChain pathChain) {
        super(name, telemetry, ticker, TimeUnit.SECONDS.toMillis(30));
        this.follower = follower;
        this.pathChain = pathChain;
    }

    @Override
    public void resetToStart() {
        super.resetToStart();

        followerHasStarted = false;
    }

    @Override
    public State doStuffAndGetNextState() {
        if (!followerHasStarted) {
            Log.d(LOG_TAG, String.format("Starting to follow %s, for state %s", pathChain, name));

            follower.followPath(pathChain);

            return this;
        }

        follower.update();

        if (follower.isBusy()) {
            return this;
        }

        if (isTimedOut()) {
            Log.e(LOG_TAG, String.format("State %s timed out after %d ms, returning next state", name, safetyTimeoutMillis));

            // FIXME: Stop the follower, but how?
            resetToStart();

            return nextState;
        }

        return nextState;
    }
}
