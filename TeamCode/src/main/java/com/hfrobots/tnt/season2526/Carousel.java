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

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Ticker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.NonNull;

public class Carousel {
    public static final double AUTOMATED_POWER = 0.3;
    public static final int MANUAL_ADJUST_SPEED_REDUCTION = 4;
    private final DcMotorEx carouselMotor;

    private final DigitalChannel carouselHomeLimit;

    public static final double ONE_FULL_REV = 751.8;
    public static final double ONE_THIRD_REV = ONE_FULL_REV / 3;
    public static final double ONE_SIXTH_REV = ONE_THIRD_REV / 2;

    private static double[] LAUNCH_POSITIONS = {
            0 + ONE_SIXTH_REV,
            ONE_SIXTH_REV + ONE_THIRD_REV,
            ONE_SIXTH_REV + ONE_THIRD_REV + ONE_THIRD_REV
    };

    private static double[] INDEX_POSITIONS = {
            0,
            ONE_THIRD_REV,
            ONE_THIRD_REV + ONE_THIRD_REV
    };

    private int currentLaunchIndex = 0;

    private int currentIntakeIndex = 0;

    public Carousel(final HardwareMap hardwareMap) {
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        carouselHomeLimit = hardwareMap.get(DigitalChannel.class, "carouselHome");
    }

    public boolean isBusyIndexing() {
        return carouselMotor.isBusy();
    }

    public void nextIndexForIntake() {
        currentIntakeIndex = currentIntakeIndex + 1;

        if (currentIntakeIndex > 2) {
            currentIntakeIndex = 0;
        }

        double targetPos = INDEX_POSITIONS[currentIntakeIndex];

        runToPosition(targetPos);
    }

    public void nextIndexForLaunch() {
        currentLaunchIndex = currentLaunchIndex + 1;

        if (currentLaunchIndex > 2) {
            currentLaunchIndex = 0;
        }

        double targetPos = LAUNCH_POSITIONS[currentLaunchIndex];

        runToPosition(targetPos);
    }

    private boolean runningToPosition = false;

    private void runToPosition(double targetPos) {
        carouselMotor.setTargetPosition((int) targetPos);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setPower(AUTOMATED_POWER);
        runningToPosition = true;
    }

    public void manuallyAdjust(final RangeInput carouselThrottle, final boolean unsafeIsPressed) {
        manuallyAdjust(carouselThrottle.getPosition(), unsafeIsPressed);
    }

    private void manuallyAdjust(float carouselThrottlePosition, final boolean unsafeIsPressed) {
        if (carouselThrottlePosition != 0) {
            if (runningToPosition) {
                runningToPosition = false;

                if (!unsafeIsPressed) {
                    carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }

        // Are we attempting to automatically position?
        if (runningToPosition) {
            return;
        }

        carouselThrottlePosition /= MANUAL_ADJUST_SPEED_REDUCTION;

        if (carouselThrottlePosition < 0) {
            if (!unsafeIsPressed) {
                if (!carouselHomeLimit.getState()) {
                    Log.d(LOG_TAG, "Carousel is at home and throttle is " + carouselThrottlePosition);

                    carouselMotor.setPower(0);
                    carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    return;
                }
            } else {
                Log.d(LOG_TAG, "Unsafe pressed, not auto-homing");
            }
        }

        if (!unsafeIsPressed) {
            maybeSetRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            maybeSetRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        carouselMotor.setPower(carouselThrottlePosition);
    }

    private void maybeSetRunMode(final DcMotor.RunMode runMode) {
        if (carouselMotor.getMode() != runMode) {
            carouselMotor.setMode(runMode);
        }
    }

    private boolean isAtHomePosition() {
        return !carouselHomeLimit.getState();
    }

    public class NextLaunchIndexState extends StopwatchTimeoutSafetyState {
        private boolean initialized = false;
        protected NextLaunchIndexState(final Telemetry telemetry, @NonNull final Ticker ticker) {
            super("Carousel indexing", telemetry, ticker, 10_000);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (!initialized) {
                nextIndexForLaunch();
                initialized = true;

                return this;
            }

            if (isTimedOut()) {
                Log.e(LOG_TAG, "Timed out waiting to index, moving to next state");

                manuallyAdjust(0, true);
                resetToStart();

                return nextState;
            }

            if (!isBusyIndexing()) {
                Log.d(LOG_TAG, "Indexing complete, moving to next state");

                resetToStart();

                return nextState;
            }

            return this;
        }

        @Override
        public void resetToStart() {
            super.resetToStart();

            initialized = false;
        }
    }

    public class NextIntakeIndexState extends StopwatchTimeoutSafetyState {
        private boolean initialized = false;
        protected NextIntakeIndexState(final Telemetry telemetry, @NonNull final Ticker ticker) {
            super("Carousel indexing", telemetry, ticker, 10_000);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (!initialized) {
                nextIndexForIntake();
                initialized = true;

                return this;
            }

            if (isTimedOut()) {
                Log.e(LOG_TAG, "Timed out waiting to index, moving to next state");

                manuallyAdjust(0, true);
                resetToStart();

                return nextState;
            }

            if (!isBusyIndexing()) {
                Log.d(LOG_TAG, "Indexing complete, moving to next state");

                resetToStart();

                return nextState;
            }

            return this;
        }

        @Override
        public void resetToStart() {
            super.resetToStart();

            initialized = false;
        }
    }

    public class HomeLocationState extends StopwatchTimeoutSafetyState {
        @Override
        public void resetToStart() {
            super.resetToStart();
            manuallyAdjust(0, true);
        }

        protected HomeLocationState(final Telemetry telemetry, @NonNull final Ticker ticker) {
            super("Carousel homing", telemetry, ticker, 7_000);
        }

        @Override
        public State doStuffAndGetNextState() {
            manuallyAdjust(-.2F, false); // head towards hard stop

            if (!isAtHomePosition()) {
                return this;
            }

            if (isTimedOut()) {
                Log.e(LOG_TAG, "Timed out while homing carousel");

                resetToStart();

                return nextState;
            }

            Log.d(LOG_TAG, "Carousel has completed homing");

            return nextState;
        }
    }
}