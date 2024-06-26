/**
 Copyright (c) 2018 HF Robotics (http://www.hfrobots.com)
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
 **/

package com.hfrobots.tnt.corelib.drive.mecanum;

import android.util.Log;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.NonNull;

/**
 * A TNT State Machine state that will follow a RoadRunner trajectory
 * using motion profiles for the given MecanumDrive.
 */
public abstract class TrajectoryFollowerState extends StopwatchTimeoutSafetyState {
    protected Trajectory trajectory;

    private boolean initialized;

    protected final RoadRunnerMecanumDriveBase driveBase;

    public TrajectoryFollowerState(@NonNull String name,
                                   @NonNull Telemetry telemetry,
                                   @NonNull RoadRunnerMecanumDriveBase driveBase,
                                   @NonNull Ticker ticker,
                                   long safetyTimeoutMillis) {
        super(name, telemetry, ticker, safetyTimeoutMillis);

        this.driveBase = driveBase;
    }

    @Override
    public State doStuffAndGetNextState() {
        if (isTimedOut()) {
            Log.d(Constants.LOG_TAG, getName() + " timed out, moving to next state");

            return nextState;
        }

        if (!initialized) {
            trajectory = createTrajectory();

            if (trajectory == null) {
                Log.d(Constants.LOG_TAG, "No trajectory, nothing to follow");
                return nextState;
            }

            driveBase.followTrajectory(trajectory);

            initialized = true;
        }

        if (driveBase.isBusy()) {
            driveBase.update();

            return this;
        }

        return nextState;
    }

    /**
     * We don't create the trajectory until the state machine runs. This allows autonomous
     * to have one set of code, that does different things depending on what alliance has been
     * selected - which in most FTC games, is simply a mirror image of left/right of the
     * other alliance routes.
     *
     * @return A Road Runner trajectory to move the robot for this State in the state machine
     */
    protected abstract Trajectory createTrajectory();
}
