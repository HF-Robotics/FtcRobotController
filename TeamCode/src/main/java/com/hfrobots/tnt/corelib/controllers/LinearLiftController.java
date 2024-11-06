/*
 Copyright (c) 2023 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.corelib.controllers;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.State;
import com.google.common.annotations.VisibleForTesting;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import lombok.Builder;
import lombok.NonNull;
import lombok.Setter;

public class LinearLiftController {
    protected static abstract class Tunables {
        protected double getAntigravityFeedForward() {
            return .24;
        }

        protected double getLiftPowerLevel() {
            return 1;
        }

        protected double getKPUpperLimit() {
            return .008;
        }

        protected double getKPLowerLimit() {
            return .0008;
        }

        protected double getOpenLoopDownPowerRatio() {
            return .02;
        }

        protected abstract int getUpperLimitEncoderPos();

        protected abstract int getLowerLimitEncoderPos();

        protected abstract int getClosedLoopStallTimeoutMillis();

        protected double getPidOutputLowerLimitMin() {
            return -.02;
        }

        double getPidOutputLowerLimitMax() {
            return 1;
        }
    }
    
    @Setter
    protected RangeInput liftThrottle;

    @Setter
    protected DebouncedButton liftUpperLimitButton;

    @Setter
    protected DebouncedButton liftLowerLimitButton;
    
    @Setter
    protected DebouncedButton liftEmergencyStopButton;

    @Setter
    protected OnOffButton limitOverrideButton;

    protected final ExtendedDcMotor liftMotor;

    protected final DigitalChannel upperLiftLimit;

    protected final DigitalChannel lowerLiftLimit;

    protected final Telemetry telemetry;

    protected State currentState;

    protected final Tunables tunables;

    protected LiftGoUpperLimitState goUpperLimitState;

    protected LiftGoLowerLimitState goLowerLimitState;

    protected LiftDownCommandState downCommandState;

    protected LiftUpCommandState upCommandState;

    protected LiftAtUpperLimitState atUpperLimitState;

    protected LiftAtLowerLimitState atLowerLimitState;

    protected LiftIdleState idleState;

    protected boolean traveling;

    @Builder
    protected LinearLiftController(final Tunables tunables,
                                final RangeInput liftThrottle,
                                final DebouncedButton liftUpperLimitButton,
                                final DebouncedButton liftLowerLimitButton,
                                final DebouncedButton liftEmergencyStopButton,
                                @NonNull final ExtendedDcMotor liftMotor,
                                final DigitalChannel upperLiftLimit,
                                final DigitalChannel lowerLiftLimit,
                                final Telemetry telemetry,
                                final OnOffButton limitOverrideButton) {
        this.tunables = tunables;
        
        this.liftThrottle = liftThrottle;

        this.liftUpperLimitButton = liftUpperLimitButton;
        
        this.liftLowerLimitButton = liftLowerLimitButton;

        this.liftEmergencyStopButton = liftEmergencyStopButton;

        this.liftMotor = liftMotor;

        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftMotor.resetLogicalEncoderCount();
        
        this.upperLiftLimit = upperLiftLimit;

        this.lowerLiftLimit = lowerLiftLimit;

        this.telemetry = telemetry;

        this.limitOverrideButton = limitOverrideButton;

        setupStateMachine(telemetry);
    }

    protected int getLogicalEncoderPosition() {
        return liftMotor.getCurrentPosition();
    }

    protected State preHandleButtons() {
        return null;
    }

    protected State postHandleButtons(final State buttonState) {
        return null;
    }

    protected void setupStateMachine(Telemetry telemetry) {
        goUpperLimitState = new LiftGoUpperLimitState(telemetry);

        goLowerLimitState = new LiftGoLowerLimitState(telemetry);

        downCommandState = new LiftDownCommandState(telemetry);

        upCommandState = new LiftUpCommandState(telemetry);

        atUpperLimitState = new LiftAtUpperLimitState(telemetry);

        atLowerLimitState = new LiftAtLowerLimitState(telemetry);

        idleState = new LiftIdleState(telemetry);

        currentState = atLowerLimitState;
    }

    protected void stopLift() {
        liftMotor.setPower(0);
    }

    protected void liftUp() {
        traveling = true;

        if (liftThrottle != null) {
            liftMotor.setPower(Math.abs(liftThrottle.getPosition()));
        }
    }

    protected void liftDown() {
        traveling = true;

        if (liftThrottle != null) {
            liftMotor.setPower(tunables.getOpenLoopDownPowerRatio() * (-Math.abs(liftThrottle.getPosition())));
        }
    }

    public void periodicTask() {
        String currentStateName = currentState.getName();

        if (currentStateName != null) {
            currentStateName = currentStateName.replace("Lift-", "");
        } else {
            currentStateName = "Unk";
        }

        telemetry.addData("SM: ", "e: " + currentStateName + " b: " +
                (nullSafeIsPressed(limitOverrideButton) ? "!" : ""));

        State nextState = currentState.doStuffAndGetNextState();

        if (nextState != currentState) {
            // We've changed states alert the driving team, log for post-match analysis
            telemetry.addData("00-State", "From %s to %s", currentState, nextState);
            Log.d(LOG_TAG, String.format("State transition from %s to %s", currentState.getClass()
                    + "(" + currentState.getName() + ")", nextState.getClass() + "(" + nextState.getName() + ")"));
        }

        currentState = nextState;
    }
    
    @VisibleForTesting
    @SuppressWarnings("unused")
    public boolean currentStateIsOpenLoopUpOrDown() {
        if (currentState == null) {
            return false;
        }

        boolean currentStateIsUpCommand = currentState.getClass().getName().equals(LiftUpCommandState.class.getName());
        boolean currentStateIsDownCommand = currentState.getClass().getName().equals(LiftDownCommandState.class.getName());

        return currentStateIsUpCommand || currentStateIsDownCommand;
    }

    @VisibleForTesting
    @SuppressWarnings("unused")
    public boolean currentStateIsIdle() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(LiftIdleState.class.getName());
    }

    @VisibleForTesting
    @SuppressWarnings("unused")
    public boolean currentStateIsAtUpperLimit() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(LiftAtUpperLimitState.class.getName());
    }

    @VisibleForTesting
    @SuppressWarnings("unused")
    public boolean currentStateIsAtLowerLimit() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(LiftAtLowerLimitState.class.getName());
    }

    @VisibleForTesting
    @SuppressWarnings("unused")
    public boolean currentStateIsClosedLoopUp() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(LiftGoUpperLimitState.class.getName());
    }

    @VisibleForTesting
    @SuppressWarnings("unused")
    public boolean currentStateIsClosedLoopDown() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(LiftGoLowerLimitState.class.getName());
    }

    @VisibleForTesting
    @SuppressWarnings("unused")
    public boolean currentStateIsGoingUpOrAtUpperLimit() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(LiftGoUpperLimitState.class.getName()) ||
                currentState.getClass().getName().equals(LiftAtUpperLimitState.class.getName());
    }

    @VisibleForTesting
    @SuppressWarnings("unused")
    public boolean isAtUpperLimit() {
        return currentState.getClass().getName().equals(LiftAtUpperLimitState.class.getName());
    }

    @VisibleForTesting
    @SuppressWarnings("unused")
    public boolean isAtLowerLimit() {
       return currentState.getClass().getName().equals(LiftAtLowerLimitState.class.getName());
    }

    protected abstract class LiftBaseState extends TimeoutSafetyState {
        LiftBaseState(String name, Telemetry telemetry, long timeoutMillis) {
            super(name, telemetry, timeoutMillis);
        }

        protected State handleButtons() {
            State preState = preHandleButtons();

            if (preState != null) {
                return preState;
            }

            final State resultantState;

            if (liftThrottleIsUp()) {
                resultantState = upCommandState;
            } else if (liftThrottleIsDown()) {
                return downCommandState;
            } else if (nullSafeGetRise(liftUpperLimitButton)) {
                Log.d(LOG_TAG, "Sensed button press for auto upper limit");

                resultantState = goUpperLimitState;
            } else if (nullSafeGetRise(liftLowerLimitButton)) {
                Log.d(LOG_TAG, "Sensed button press for auto lower limit");

                resultantState = goLowerLimitState;
            } else if (nullSafeGetRise(liftEmergencyStopButton)) {
                stopLift();

                resultantState = idleState;
            } else {
                resultantState = null;
            }

            final State postOverrideState = postHandleButtons(resultantState);

            if (postOverrideState != null) {
                return postOverrideState;
            }

            return resultantState;
        }

        protected State transitionToState(State state) {
            resetTimer();

            return state;
        }
    }

    protected boolean liftThrottleIsDown() {
        if (liftThrottle == null) {
            return false;
        }

        return liftThrottle.getPosition() > 0;
    }

    protected class LiftIdleState extends LiftBaseState {

        protected LiftIdleState(Telemetry telemetry) {
            super("Elev-idle", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null) {
                return fromButtonState;
            }

            liftMotor.setPower(tunables.getAntigravityFeedForward());

            return this; // FIXME: THis isn't correct - what should be returned if nothing has changed?
        }
    }

    protected abstract class LiftClosedLoopState extends LiftBaseState {
        protected PidController pidController;

        protected boolean initialized = false;

        protected LiftClosedLoopState(String name, Telemetry telemetry, long timeoutMillis) {
            super(name, telemetry, timeoutMillis);
        }

        protected void setupPidController(double kP) {
            pidController = PidController.builder().setInstanceName("Lift mechanism pid-controller")
                    .setKp(kP).setAllowOscillation(false)
                    .setTolerance(45)
                    .build();
            pidController.setAbsoluteSetPoint(true);
            pidController.setOutputRange(-tunables.getLiftPowerLevel(),
                    tunables.getLiftPowerLevel());
        }

        @Override
        protected State transitionToState(State state) {
            State transition = super.transitionToState(state);

            if (pidController != null) {
                pidController.reset();
            }

            initialized = false;

            return transition;
        }
    }

    class LiftGoUpperLimitState extends LiftClosedLoopState {
        LiftGoUpperLimitState(Telemetry telemetry) {
            super("Elev-auto-top", telemetry, tunables.getClosedLoopStallTimeoutMillis());
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isTimedOut() && ! isUnsafePressed()) {
                stopLift();
                Log.e(LOG_TAG, "Timed out while going to upper limit");

                return transitionToState(idleState);
            }

            State fromButtonState = handleButtons();

            // Handle possible transitions back to open loop
            if (fromButtonState != null && fromButtonState != this) {
                Log.d(LOG_TAG, "Leaving closed loop for " + fromButtonState.getName());

                return transitionToState(fromButtonState);
            }

            if (!initialized) {
                Log.d(LOG_TAG, "Initializing PID for state " + getName());

                setupPidController(tunables.getKPUpperLimit());

                pidController.setAbsoluteSetPoint(true); // MM
                pidController.setTarget(tunables.getUpperLimitEncoderPos(),
                        liftMotor.getCurrentPosition());

                initialized = true;
            }

            // closed loop control based on motor encoders

            double pidOutput = pidController.getOutput(liftMotor.getCurrentPosition());

            if (pidController.isOnTarget()) {
                stopLift();

                Log.d(LOG_TAG, "Lift reached upper target");

                initialized = false;

                return transitionToState(atUpperLimitState);
            }

            if (limitSwitchIsOn(upperLiftLimit)) {
                stopLift();

                initialized = false;

                return transitionToState(atUpperLimitState);
            }

            Log.d(LOG_TAG, "Lift setting power via PID to: " + pidOutput);

            double ANTIGRAVITY_OVERCOME_FEED_FORWARD = .33;

            if (pidOutput < ANTIGRAVITY_OVERCOME_FEED_FORWARD) {
                pidOutput = ANTIGRAVITY_OVERCOME_FEED_FORWARD;
            }

            liftMotor.setPower(pidOutput);

            return this;
        }

        @Override
        protected State transitionToState(State state) {
            return super.transitionToState(state);
        }
    }


    protected class LiftGoLowerLimitState extends LiftClosedLoopState {
        protected LiftGoLowerLimitState(Telemetry telemetry) {
            super("Lift-auto-bot", telemetry, tunables.getClosedLoopStallTimeoutMillis());
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isTimedOut() && ! isUnsafePressed()) {
                stopLift();
                Log.e(LOG_TAG, "Timed out while going to low limit");

                return transitionToState(idleState);
            }

            State fromButtonState = handleButtons();

            // Handle possible transitions back to open loop
            if (fromButtonState != null && fromButtonState != this) {
                Log.d(LOG_TAG, "Leaving closed loop for " + fromButtonState.getName());

                return transitionToState(fromButtonState);
            }

            if (!initialized) {
                initialize();

                initialized = true;
            }

            // closed loop control based on motor encoders

            int currentPosition = liftMotor.getCurrentPosition();

            double pidOutput = pidController.getOutput(currentPosition);

            if (pidController.isOnTarget()) {
                Log.d(LOG_TAG, "Lift reached lower target");

                liftMotor.setPower(tunables.getAntigravityFeedForward());

                reachedLowerLimit();

                return transitionToState(atLowerLimitState);
            }

            if (limitSwitchIsOn(lowerLiftLimit)) {
                stopLift();

                liftMotor.resetLogicalEncoderCount();

                reachedLowerLimit();

                return transitionToState(atLowerLimitState);
            }

            liftMotor.setPower(pidOutput);

            return this;
        }

        protected void reachedLowerLimit() {

        }

        protected void initialize() {
            Log.d(LOG_TAG, "Initializing PID for state " + getName());

            setupPidController(tunables.getKPLowerLimit());

            pidController.setOutputRange(tunables.getPidOutputLowerLimitMin(),
                    tunables.getPidOutputLowerLimitMax()); // fix slamming while descending
            pidController.setAbsoluteSetPoint(true); // MM

            int currentPosition = liftMotor.getCurrentPosition();

            pidController.setTarget(tunables.getLowerLimitEncoderPos(),
                    currentPosition);
        }
    }

    class LiftAtUpperLimitState extends LiftBaseState {

        LiftAtUpperLimitState(Telemetry telemetry) {
            super("Lift-@-top", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null && fromButtonState != this &&
                    !fromButtonState.equals(goUpperLimitState) &&
                    !fromButtonState.equals(upCommandState)) {
                Log.d(LOG_TAG, getName() + " responding to buttons and transitioning to " + fromButtonState.getName());

                liftMotor.setPower(0);

                return fromButtonState;
            } else if (fromButtonState != null && isUnsafePressed()) {
                if (fromButtonState != this) {
                    Log.d(LOG_TAG, getName() + " - limits overridden - transitioning to " + fromButtonState.getName());

                    liftMotor.setPower(0);

                    return fromButtonState;
                }
            }

            // FIXME: Let's start with feed-forward only, and see if we really need a PID
            // to hold position

            liftMotor.setPower(tunables.getAntigravityFeedForward());

            return this;
        }
    }


    protected class LiftAtLowerLimitState extends LiftBaseState {

        LiftAtLowerLimitState(Telemetry telemetry) {
            super("Lift-@-bot", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null &&  fromButtonState != this &&
                    !fromButtonState.equals(goLowerLimitState) &&
                    !fromButtonState.equals(downCommandState)) {
                Log.d(LOG_TAG, getName() + " responding to buttons and transitioning to " + fromButtonState.getName());

                return fromButtonState;
            } else if (fromButtonState != null && isUnsafePressed()) {
                if (fromButtonState != this) {
                    Log.d(LOG_TAG, getName() + " - limits overridden - transitioning to " + fromButtonState.getName());

                    return fromButtonState;
                }
            }

            return this;
        }
    }

    protected boolean isUnsafePressed() {
        return nullSafeIsPressed(limitOverrideButton);
    }

    protected class LiftDownCommandState extends LiftBaseState {

        protected LiftDownCommandState(Telemetry telemetry) {
            super("Lift-cmd-dn", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        protected State transitionToState(State state) {
            traveling = false;
            return super.transitionToState(state);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (liftThrottleIsUp()) {
                    return transitionToState(upCommandState);
            } else if (nullSafeGetRise(liftUpperLimitButton)) {
                    return transitionToState(goUpperLimitState);
            } else if (nullSafeGetRise(liftLowerLimitButton)){
                    return transitionToState(goLowerLimitState);
            } else if (nullSafeGetRise(liftEmergencyStopButton)) {
                stopLift();

                return transitionToState(idleState);
            }

            if (limitSwitchIsOn(lowerLiftLimit)) {
                stopLift();

                liftMotor.resetLogicalEncoderCount();

                return transitionToState(atLowerLimitState);
            } else if (!liftThrottleIsUp() && !liftThrottleIsDown()) {
                Log.d(LOG_TAG, "Lift - down command button released");
                stopLift();

                return transitionToState(idleState);
            }

            liftDown();

            return this;
        }
    }

    protected boolean liftThrottleIsUp() {
        if (liftThrottle == null) {
            return false;
        }

        return liftThrottle.getPosition() < 0;
    }

    protected class LiftUpCommandState extends LiftBaseState {

        protected LiftUpCommandState(Telemetry telemetry) {
            super("Lift-cmd-up", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            if (liftThrottleIsDown()) {
                return transitionToState(downCommandState);
            } else if (nullSafeGetRise(liftUpperLimitButton)) {
                return transitionToState(goUpperLimitState);
            } else if (nullSafeGetRise(liftLowerLimitButton)){
                return transitionToState(goLowerLimitState);
            } else if (nullSafeGetRise(liftEmergencyStopButton)) {
                stopLift();

                return transitionToState(idleState);
            }

            if (limitSwitchIsOn(upperLiftLimit)) {
                stopLift();

                return transitionToState(atUpperLimitState);
            }

            if (!isUnsafePressed()) {
                if (liftMotor.getCurrentPosition() > tunables.getUpperLimitEncoderPos()) {
                    stopLift();

                    return transitionToState(atUpperLimitState);
                }
            }

            if (!liftThrottleIsUp()) {
                Log.d(LOG_TAG, "Lift - up command button released");
                stopLift();

                return transitionToState(idleState);
            }

            liftUp();

            return this;
        }

        protected State transitionToState(State state) {
            traveling = false;
            return super.transitionToState(state);
        }
    }

    private boolean nullSafeGetRise(final DebouncedButton debouncedButton) {
        return debouncedButton != null && debouncedButton.getRise();
    }

    private boolean nullSafeIsPressed(final OnOffButton onOffButton) {
        return onOffButton != null && onOffButton.isPressed();
    }

    private static boolean limitSwitchIsOn(final DigitalChannel limitSwitch) {
        return limitSwitch != null && !limitSwitch.getState(); // Limit switches false -> on
    }
}
