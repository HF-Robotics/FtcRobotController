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
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

import lombok.NonNull;

public class PedroRelativeTurnState extends State {
    private final Follower follower;

    private final Supplier<Double> degreesToTurn;

    private boolean followerHasStarted = false;

    /**
     * Turns relative to the current pose, if the supplier of the degrees returns
     * null, the transition to the next state will happen immediately.
     * <p>
     * Positive values for degreesToTurn will turn CCW around the Z axis.
     */
    public PedroRelativeTurnState(@NonNull String name, Telemetry telemetry, final Follower follower, Supplier<Double> degreesToTurn) {
        super(name, telemetry);
        this.follower = follower;
        this.degreesToTurn = degreesToTurn;
    }

    @Override
    public void resetToStart() {
        followerHasStarted = false;
    }

    @Override
    public State doStuffAndGetNextState() {
        if (!followerHasStarted) {
            Double headingChangeInDegrees = degreesToTurn.get();

            if (headingChangeInDegrees != null) {
                Log.d(LOG_TAG, String.format("Starting to turn %s degrees, for state %s", degreesToTurn.get().toString(), name));

                final double headingChangeInRadians = Math.toRadians(headingChangeInDegrees);

                follower.turn(Math.abs(headingChangeInRadians), headingChangeInRadians > 0);

                followerHasStarted = true;

                return this;
            } else {
                return nextState;
            }
        }

        follower.update();

        if (follower.isBusy()) {
            return this;
        }

        return nextState;
    }
}
