/*
 Copyright (c) 2025 The Tech Ninja Team (https://ftc9929.com)

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

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
