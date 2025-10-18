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

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.state.State;
import com.google.common.base.Ticker;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

import lombok.NonNull;

public class PedroFollowerState extends State {
    private final Follower follower;

    private final Supplier<PathChain> pathChainSupplier;

    private boolean followerHasStarted = false;

    public PedroFollowerState(@NonNull String name, Telemetry telemetry, final Follower follower, final Path path) {
        this(name, telemetry, follower, new PathChain(path));
    }

    public PedroFollowerState(@NonNull String name, Telemetry telemetry, final Follower follower, final PathChain pathChain) {
        this(name, telemetry, follower, () -> pathChain);
    }

    /**
     * Use this when you need to dynamically create the path/patch chain based on the robot's current
     * state while running auto, not something you can pre-plan (for example, when the beginning
     * of the path needs to be from the current pose).
     */
    public PedroFollowerState(@NonNull String name, Telemetry telemetry, final Follower follower, final Supplier<PathChain> pathChainSupplier) {
        super(name, telemetry);
        this.follower = follower;
        this.pathChainSupplier = pathChainSupplier;
    }

    @Override
    public void resetToStart() {
        followerHasStarted = false;
    }

    @Override
    public State doStuffAndGetNextState() {
        if (!followerHasStarted) {
            final PathChain pathChain = pathChainSupplier.get();

            Log.d(LOG_TAG, String.format("Starting to follow %s, for state %s", pathChain, name));

            follower.followPath(pathChain);

            followerHasStarted = true;

            return this;
        }

        follower.update();

        if (follower.isBusy()) {
            return this;
        }

        return nextState;
    }
}
