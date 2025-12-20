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
import com.google.common.collect.Maps;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Map;

import lombok.NonNull;

public class GenevaCarousel {
    public static final double AUTOMATED_POWER = 1;

    public static final int MANUAL_ADJUST_SPEED_REDUCTION = 2;

    private final DcMotorEx carouselMotor;

    private final boolean limitSwitchIsWorking = true;

    private final DigitalChannel launchPositionDetection;

    public static final double ONE_FULL_REV = 384.5; // Yellow Jacket 435RPM

    public enum ArtifactColor { GREEN, PURPLE };

    private final Map<ArtifactColor, Integer> artifactsToPositions = Maps.newHashMap();

    private LynxI2cColorRangeSensor artifactColorSensor1;
    private LynxI2cColorRangeSensor artifactColorSensor2;

    private final Telemetry telemetry;

    // FIXME: Must always start in launch position - until we fix limit switch!
    private boolean isInLaunchPosition = true;

    public GenevaCarousel(final HardwareMap hardwareMap, Telemetry telemetry) {
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        this.telemetry = telemetry;
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launchPositionDetection = hardwareMap.get(DigitalChannel.class, "launchPositionDetection");

        try {
            artifactColorSensor1 = hardwareMap.get(LynxI2cColorRangeSensor.class, "artifactColorSensor1");
            artifactColorSensor2 = hardwareMap.get(LynxI2cColorRangeSensor.class, "artifactColorSensor2");
        } catch (Exception ex) {
            Log.e(LOG_TAG, "Unable to initialize artifact color sensors", ex);
            artifactColorSensor1 = null;
            artifactColorSensor2 = null;
        }
    }

    public boolean isBusyIndexing() {
        return carouselMotor.isBusy();
    }

    public void nextIndexForIntake() {
        if (notDoneAdvancing()) {
            return;
        }

        detectArtifactInCurrentIntake();

        int currentEncoderPosition = carouselMotor.getCurrentPosition();

        final double targetPos;

        if (isInLaunchPosition()) {
            targetPos = currentEncoderPosition + ONE_FULL_REV;
        } else {
           targetPos = currentEncoderPosition + (ONE_FULL_REV * 2);
        }

        runToPosition(targetPos);

        isInLaunchPosition = false;

    }

    private boolean notDoneAdvancing() {

        if (DcMotor.RunMode.RUN_TO_POSITION != carouselMotor.getMode()) {
            return false;
        }

        int currentEncoderCount = carouselMotor.getCurrentPosition();
        int targetEncoderCount = carouselMotor.getTargetPosition();
        int targetTolerance = carouselMotor.getTargetPositionTolerance();

        int difference = Math.abs(currentEncoderCount - targetEncoderCount);

        Log.d(LOG_TAG, "Geneva drive: ce, te, tt, diff: " + currentEncoderCount + ", " + targetEncoderCount + ", " + targetTolerance + ", " + difference);

        return difference >= targetTolerance;
    }

    private void detectArtifactInCurrentIntake() {
        if (isInLaunchPosition()) {
            return;
        }

        if (noUsableColorSensors()) {
            telemetry.addData("Carousel", "no color sensors!");

            return;
        }

        final LynxI2cColorRangeSensor closestColorSensor;

        if (artifactColorSensor1.getDistance(DistanceUnit.MM) < artifactColorSensor2.getDistance(DistanceUnit.MM)) {
            closestColorSensor = artifactColorSensor1;
        } else {
            closestColorSensor = artifactColorSensor2;
        }

        int currentCarouselPosition = carouselMotor.getCurrentPosition();

        if (weSensedAGreenArtifact(closestColorSensor)) {
            // FIXME: We need to "erase" anything we've launched
            artifactsToPositions.put(ArtifactColor.GREEN, currentCarouselPosition);
            telemetry.addData("Carousel", "Green artifact detected");
        } else if (weSensedAPurpleArtifact(closestColorSensor)) {
            artifactsToPositions.put(ArtifactColor.PURPLE, currentCarouselPosition);
            telemetry.addData("Carousel", "Purple artifact detected");
        } else {
            // Probably should alert the operator?
            telemetry.addData("Carousel", "No artifact detected");
        }
    }

    private boolean noUsableColorSensors() {
        return artifactColorSensor2 == null || artifactColorSensor1 == null;
    }

    private boolean weSensedAPurpleArtifact(LynxI2cColorRangeSensor closestColorSensor) {
        int blue = closestColorSensor.blue();
        int red = closestColorSensor.red();
        int green = closestColorSensor.green();

        boolean sensedPurple = blue > green && red > green;

        return sensedPurple;
    }

    private boolean weSensedAGreenArtifact(LynxI2cColorRangeSensor closestColorSensor) {
        int blue = closestColorSensor.blue();
        int red = closestColorSensor.red();
        int green = closestColorSensor.green();

        boolean sensedGreen = green > red;

        return sensedGreen;
    }

    public boolean isInLaunchPosition() {
        if (!limitSwitchIsWorking) {
            return isInLaunchPosition;
        }

       return !launchPositionDetection.getState();
    }

    public void nextIndexForLaunch() {
        if (notDoneAdvancing()) {
            Log.d(LOG_TAG, "Not done advancing silly puppy, ignoring you");
            return;
        }

        int currentEncoderPosition = carouselMotor.getCurrentPosition();

        final double targetPos;

        if (isInLaunchPosition()) {
            targetPos = currentEncoderPosition + (ONE_FULL_REV * 2);
        } else {
            targetPos = currentEncoderPosition + ONE_FULL_REV;
        }

        runToPosition(targetPos);
        isInLaunchPosition = true;
    }

    private boolean runningToPosition = false;

    private void runToPosition(double targetPos) {
        Log.d(LOG_TAG, "Attempting to run to position:" + targetPos);
        carouselMotor.setTargetPosition((int) targetPos);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setPower(AUTOMATED_POWER);
        runningToPosition = true;
    }

    public void manuallyAdjust(final RangeInput carouselThrottle, final boolean unsafeIsPressed) {
        manuallyAdjust(carouselThrottle.getPosition(), unsafeIsPressed);
    }

    protected void manuallyAdjust(float carouselThrottlePosition, final boolean unsafeIsPressed) {
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
                // FIXME: Do we want to stop when we reach a launch position?
            } else {
                //Log.d(LOG_TAG, "Unsafe pressed, not auto-homing");
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
}