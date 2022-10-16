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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Deque;
import java.util.LinkedList;
import java.util.List;

import lombok.AllArgsConstructor;
import lombok.NonNull;

/**
 * A TNT State Machine state that will follow a RoadRunner trajectory
 * using motion profiles for the given MecanumDrive.
 */
public abstract class MultipleTrajectoriesFollowerState extends StopwatchTimeoutSafetyState {

    @AllArgsConstructor
    private static class NamedTrajectoryProvider {
        final String trajectoryProviderName;

        final Function<TrajectoryBuilder, TrajectoryBuilder> trajectoryProvider;
    }

    @AllArgsConstructor
    private static class NamedTrajectory {
        final String trajectoryName;

        final Trajectory trajectory;
    }

    protected final Deque<NamedTrajectoryProvider> trajectoryProviders;

    protected NamedTrajectory pendingNamedTrajectory;

    protected NamedTrajectory currentNamedTrajectory;

    private boolean initialized;

    protected final RoadRunnerMecanumDriveBase driveBase;

    public MultipleTrajectoriesFollowerState(@NonNull String name,
                                             @NonNull Telemetry telemetry,
                                             @NonNull RoadRunnerMecanumDriveBase driveBase,
                                             @NonNull Ticker ticker,
                                             long safetyTimeoutMillis) {
        super(name, telemetry, ticker, safetyTimeoutMillis);

        this.driveBase = driveBase;
        trajectoryProviders = new LinkedList<>();
    }

    @Override
    public String getName() {
        final String baseName = super.getName();

        if (currentNamedTrajectory == null) {
            return baseName;
        }

        return baseName + " - " + currentNamedTrajectory.trajectoryName;
    }

    @Override
    public State doStuffAndGetNextState() {
        if (isTimedOut()) {
            Log.d(Constants.LOG_TAG, getName() + " timed out, moving to next state");

            return nextState;
        }

        if (!initialized) {
            createTrajectoryProviders();

            if (trajectoryProviders.isEmpty()) {
                Log.d(Constants.LOG_TAG, "No trajectories, nothing to follow");

                return nextState;
            }

            initialized = true;
        }

        if (driveBase.isBusy()) {
            // RoadRunner is following a trajectory
            driveBase.update();
            return this;
        }

        if (pendingNamedTrajectory == null) {
            if (trajectoryProviders.isEmpty()) {
                Log.d(Constants.LOG_TAG, "No more trajectories for this state");

                return nextState;
            }

            NamedTrajectoryProvider nmp = trajectoryProviders.removeFirst();

            Function<TrajectoryBuilder, TrajectoryBuilder> trajectoryProvider
                    = nmp.trajectoryProvider;

            final Pose2d endOfPriorTrajectory;

            if (currentNamedTrajectory == null) {
                endOfPriorTrajectory = new Pose2d();
            } else {
                endOfPriorTrajectory = currentNamedTrajectory.trajectory.end();
            }

            Trajectory newTrajectory = trajectoryProvider.apply(driveBase.trajectoryBuilder(endOfPriorTrajectory)).build();
            pendingNamedTrajectory = new NamedTrajectory(nmp.trajectoryProviderName, newTrajectory);

            Log.d(Constants.LOG_TAG, "Switching to trajectory: " + pendingNamedTrajectory.trajectoryName);

            // Need to give a cycle through the loop() in the OpMode
            // when we switch trajectories in case we're bulk reading/caching
            // sensor reads, otherwise RoadRunner gets confused
            return this;
        }

        if (pendingNamedTrajectory != null) {
            currentNamedTrajectory = pendingNamedTrajectory;
            pendingNamedTrajectory = null;
            Log.d(Constants.LOG_TAG, "Starting to run trajectory: " + currentNamedTrajectory.trajectoryName);
            driveBase.followTrajectoryAsync(currentNamedTrajectory.trajectory);
            driveBase.update();

            return this;
        }


        if (trajectoryProviders.isEmpty()) {
            // No more trajectories to follow
            Log.d(Constants.LOG_TAG, "No more trajectories for this state");

            return nextState;
        }

        return this;
    }

    /**
     * We don't create the trajectories until the state machine runs. This allows autonomous
     * to have one set of code, that does different things depending on what alliance has been
     * selected - which in most FTC games, is simply a mirror image of left/right of the
     * other alliance routes, or depending on sensing of randomized game elements at the
     * beginning of auto.
     *
     * When implementing this method, call addTrajectoryProvider() for each trajectory
     * that should be followed in order when running this state.
     */
    protected abstract void createTrajectoryProviders();

    protected void addTrajectoryProvider(@NonNull String name, @NonNull Function<TrajectoryBuilder, TrajectoryBuilder> trajectoryProvider) {
        trajectoryProviders.add(new NamedTrajectoryProvider(name, trajectoryProvider));
    }
}
