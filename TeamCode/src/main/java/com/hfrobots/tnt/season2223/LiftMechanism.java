/*
 Copyright (c) 2022 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.season2223;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.State;
import com.google.common.annotations.VisibleForTesting;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import lombok.Builder;
import lombok.Setter;

public class LiftMechanism {
    private static final double LIFT_POWER_LEVEL = 1;

    private static final double K_P_UPPER_LIMIT = .008;

    private static final double K_P_LOWER_LIMIT = .0008;

    private static final double K_P_SMALL_JUNCTION = .004;

    private static final double K_P_MEDIUM_JUNCTION = .004;

    private static final double ANTIGRAVITY_FEED_FORWARD = .24; // FIXME: needs adjusted for new lift setup

    private static final int LIFT_SMALL_JUNCTION_ENCODER_POS = 473; // FIXME: Needs tuned

    private static final int LIFT_MEDIUM_JUNCTION_ENCODER_POS = 857; // FIXME: Needs tuned

    private static final int LIFT_UPPER_LIMIT_ENCODER_POS = 1224; // FIXME: Needs tuned

    private static final int LIFT_LOWER_LIMIT_ENCODER_POS = 0;

    private static final int AUTO_STALL_TIMEOUT_SECONDS = 8;

    private static final double OPEN_LOOP_DOWN_POWER_RATIO = .02;

    private static final double PID_OUTPUT_LOWER_LIMIT_MIN = -.02; // FIXME: Needs tuned - or disabled
    private static final double PID_OUTPUT_LOWER_LIMIT_MAX = 1;

    public static LiftMechanismBuilder builderFromHardwareMap(final HardwareMap hardwareMap,
                                                final Telemetry telemetry) {
        DigitalChannel lowerLimit = hardwareMap.get(DigitalChannel.class, "lowLimitSwitch");
        DigitalChannel higherLimit = hardwareMap.get(DigitalChannel.class, "highLimitSwitch");

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        return LiftMechanism.builder().liftMotor(NinjaMotor.asNeverest20(liftMotor))
                .lowerLiftLimit(lowerLimit)
                .upperLiftLimit(higherLimit)
                .telemetry(telemetry);
    }

    @Setter
    private RangeInput liftThrottle;

    @Setter
    private DebouncedButton liftUpperLimitButton;

    @Setter
    private DebouncedButton liftLowerLimitButton;

    @Setter
    private DebouncedButton liftGoSmallButton;

    @Setter
    private DebouncedButton liftGoMediumButton;

    @Setter
    private DebouncedButton liftEmergencyStopButton;

    @Setter
    private OnOffButton limitOverrideButton;

    private final ExtendedDcMotor liftMotor;

    private final DigitalChannel upperLiftLimit;

    private final DigitalChannel lowerLiftLimit;

    private final Telemetry telemetry;

    private State currentState;

    private final Gripper gripper;

    @Builder
    public LiftMechanism(final RangeInput liftThrottle,
                         final DebouncedButton liftUpperLimitButton,
                         final DebouncedButton liftLowerLimitButton,
                         final DebouncedButton liftGoSmallButton,
                         final DebouncedButton liftGoMediumButton,
                         final DebouncedButton liftEmergencyStopButton,
                         final ExtendedDcMotor liftMotor,
                         final DigitalChannel upperLiftLimit,
                         final DigitalChannel lowerLiftLimit,
                         final Gripper gripper,
                         final Telemetry telemetry,
                         final OnOffButton limitOverrideButton) {
        this.liftThrottle = liftThrottle;

        this.liftUpperLimitButton = liftUpperLimitButton;

        this.liftGoSmallButton = liftGoSmallButton;

        this.liftGoMediumButton = liftGoMediumButton;

        this.liftLowerLimitButton = liftLowerLimitButton;

        this.liftEmergencyStopButton = liftEmergencyStopButton;

        this.liftMotor = liftMotor;

        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftMotor.resetLogicalEncoderCount();

        this.gripper = gripper;

        this.upperLiftLimit = upperLiftLimit;

        this.lowerLiftLimit = lowerLiftLimit;

        this.telemetry = telemetry;

        this.limitOverrideButton = limitOverrideButton;

        setupStateMachine(telemetry);
    }

    private void setupStateMachine(Telemetry telemetry) {
        LiftGoUpperLimitState goUpperLimitState = new LiftGoUpperLimitState(telemetry);

        LiftGoLowerLimitState goLowerLimitState = new LiftGoLowerLimitState(telemetry);

        LiftGoSmallJunctionState goSmallJunctionState = new LiftGoSmallJunctionState(telemetry);

        LiftGoMediumJunctionState goMediumJunctionState = new LiftGoMediumJunctionState(telemetry);

        LiftDownCommandState downCommandState = new LiftDownCommandState(telemetry);

        LiftUpCommandState upCommandState = new LiftUpCommandState(telemetry);

        LiftAtUpperLimitState atUpperLimitState = new LiftAtUpperLimitState(telemetry);

        LiftAtLowerLimitState atLowerLimitState = new LiftAtLowerLimitState(telemetry);

        LiftIdleState idleState = new LiftIdleState(telemetry);

        goUpperLimitState.setAllTransitionToStates(goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
         upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        goSmallJunctionState.setAllTransitionToStates(goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        goMediumJunctionState.setAllTransitionToStates(goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        goLowerLimitState.setAllTransitionToStates(goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        downCommandState.setAllTransitionToStates(goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        upCommandState.setAllTransitionToStates(goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        atUpperLimitState.setAllTransitionToStates(goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        atLowerLimitState.setAllTransitionToStates(goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        idleState.setAllTransitionToStates(goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        currentState = atLowerLimitState;
    }

    protected void stopLift() {
        liftMotor.setPower(0);
    }

    protected void liftUp() {
        liftMotor.setPower(Math.abs(liftThrottle.getPosition()));
    }

    protected void liftDown() {
        liftMotor.setPower(OPEN_LOOP_DOWN_POWER_RATIO * (-Math.abs(liftThrottle.getPosition())));
    }

    public void periodicTask() {
        String currentStateName = currentState.getName();

        if (currentStateName != null) {
            currentStateName = currentStateName.replace("Lift-", "");
        } else {
            currentStateName = "Unk";
        }

        telemetry.addData("SM: ", "e: " + currentStateName + " b: " +
                (limitOverrideButton.isPressed() ? "!" : ""));

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

    abstract class LiftBaseState extends TimeoutSafetyState {
        LiftGoUpperLimitState goUpperLimitState;

        LiftGoSmallJunctionState goSmallJunctionState;

        LiftGoMediumJunctionState goMediumJunctionState;

        LiftGoLowerLimitState goLowerLimitState;

        LiftDownCommandState downCommandState;

        LiftUpCommandState upCommandState;

        LiftAtUpperLimitState atUpperLimitState;

        LiftAtLowerLimitState atLowerLimitState;

        LiftIdleState idleState;

        LiftBaseState(String name, Telemetry telemetry, long timeoutMillis) {
            super(name, telemetry, timeoutMillis);
        }

        protected void setAllTransitionToStates(final LiftGoUpperLimitState goUpperLimitState,
                final LiftGoSmallJunctionState goSmallJunctionState,
                final LiftGoMediumJunctionState goMediumJunctionState,
                final LiftGoLowerLimitState goLowerLimitState,
                final LiftDownCommandState downCommandState,
                final LiftUpCommandState upCommandState,
                final LiftAtLowerLimitState atLowerLimitState,
                final LiftAtUpperLimitState atUpperLimitState,
                final LiftIdleState idleState) {
            this.goLowerLimitState = goLowerLimitState;

            this.goSmallJunctionState = goSmallJunctionState;

            this.goMediumJunctionState = goMediumJunctionState;

            this.goUpperLimitState = goUpperLimitState; // MM

            this.downCommandState = downCommandState;

            this.upCommandState = upCommandState;

            this.atLowerLimitState = atLowerLimitState;

            this.atUpperLimitState = atUpperLimitState;

            this.idleState = idleState;

            // FIXME: Assert that everything that is required, has been set! (there was a bug hiding in this code!)

        }

        protected State handleButtons() {
            if (liftThrottleIsUp()) {
                return upCommandState;
            } else if (liftThrottleIsDown()) {
                return downCommandState;
            } else if (liftUpperLimitButton.getRise()) {
                Log.d(LOG_TAG, "Sensed button press for auto upper limit");

                return goUpperLimitState;
            } else if (liftLowerLimitButton.getRise()) {
                Log.d(LOG_TAG, "Sensed button press for auto lower limit");

                return goLowerLimitState;
            } else if (liftGoSmallButton.getRise()) {
                Log.d(LOG_TAG, "Sensed button press for auto small limit");

                return goSmallJunctionState;
            } else if (liftGoMediumButton.getRise()) {
                Log.d(LOG_TAG, "Sensed button press for medium small limit");

                return goMediumJunctionState;
            } else if (liftEmergencyStopButton.getRise()) {
                stopLift();

                return idleState;
            }

            return null;
        }

        protected State transitionToState(State state) {
            resetTimer();

            return state;
        }
    }

    private boolean liftThrottleIsDown() {
        return liftThrottle.getPosition() > 0;
    }

    class LiftIdleState extends LiftBaseState {

        LiftIdleState(Telemetry telemetry) {
            super("Elev-idle", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null) {
                return fromButtonState;
            }

            liftMotor.setPower(ANTIGRAVITY_FEED_FORWARD);

            return this; // FIXME: THis isn't correct - what should be returned if nothing has changed?
        }
    }

    abstract class LiftClosedLoopState extends LiftBaseState {
        PidController pidController;

        boolean initialized = false;

        LiftClosedLoopState(String name, Telemetry telemetry, long timeoutMillis) {
            super(name, telemetry, timeoutMillis);
        }

        protected void setupPidController(double kP) {
            pidController = PidController.builder().setInstanceName("Lift mechanism pid-controller")
                    .setKp(kP).setAllowOscillation(false)
                    .setTolerance(45)
                    .build();
            pidController.setAbsoluteSetPoint(true);
            pidController.setOutputRange(-LIFT_POWER_LEVEL, LIFT_POWER_LEVEL);
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
            super("Elev-auto-top", telemetry, TimeUnit.SECONDS.toMillis(AUTO_STALL_TIMEOUT_SECONDS));
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isTimedOut() && limitOverrideButton != null && !limitOverrideButton.isPressed()) {
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

                setupPidController(K_P_UPPER_LIMIT);

                pidController.setAbsoluteSetPoint(true); // MM
                pidController.setTarget(LIFT_UPPER_LIMIT_ENCODER_POS,
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


    class LiftGoLowerLimitState extends LiftClosedLoopState {

        int leavingFromPosition = LIFT_UPPER_LIMIT_ENCODER_POS;

        LiftGoLowerLimitState(Telemetry telemetry) {
            super("Lift-auto-bot", telemetry, TimeUnit.SECONDS.toMillis(AUTO_STALL_TIMEOUT_SECONDS)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isTimedOut() && limitOverrideButton != null && !limitOverrideButton.isPressed()) {
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
                Log.d(LOG_TAG, "Initializing PID for state " + getName());

                setupPidController(K_P_LOWER_LIMIT);

                pidController.setOutputRange(PID_OUTPUT_LOWER_LIMIT_MIN, PID_OUTPUT_LOWER_LIMIT_MAX); // fix slamming while descending
                pidController.setAbsoluteSetPoint(true); // MM
                int currentPosition = liftMotor.getCurrentPosition();

                pidController.setTarget(LIFT_LOWER_LIMIT_ENCODER_POS,
                        currentPosition);

                leavingFromPosition = currentPosition;

                gripper.close();

                initialized = true;
            }

            // closed loop control based on motor encoders

            int currentPosition = liftMotor.getCurrentPosition();

            double pidOutput = pidController.getOutput(currentPosition);

            if (pidController.isOnTarget()) {
                Log.d(LOG_TAG, "Lift reached lower target");

                liftMotor.setPower(ANTIGRAVITY_FEED_FORWARD);

                gripper.open();

                return transitionToState(atLowerLimitState);
            }

            if (limitSwitchIsOn(lowerLiftLimit)) {
                stopLift();

                liftMotor.resetLogicalEncoderCount();

                gripper.open();

                return transitionToState(atLowerLimitState);
            }

            liftMotor.setPower(pidOutput);

            return this;
        }
    }

    class LiftGoSmallJunctionState extends LiftClosedLoopState {

        LiftGoSmallJunctionState(Telemetry telemetry) {
            super("Lift-auto-small", telemetry, TimeUnit.SECONDS.toMillis(AUTO_STALL_TIMEOUT_SECONDS)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isTimedOut() && limitOverrideButton != null && !limitOverrideButton.isPressed()) {
                stopLift();
                Log.e(LOG_TAG, "Timed out while going to small junction");

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

                setupPidController(K_P_SMALL_JUNCTION);

                pidController.setOutputRange(-1, 1); // fix bouncing while descending
                pidController.setAbsoluteSetPoint(true); // MM

                pidController.setTarget(LIFT_SMALL_JUNCTION_ENCODER_POS,
                        liftMotor.getCurrentPosition());

                initialized = true;
            }

            // closed loop control based on motor encoders

            double pidOutput = pidController.getOutput(liftMotor.getCurrentPosition());

            if (pidController.isOnTarget()) {
                Log.d(LOG_TAG, "Lift reached small target");

                stopLift();

                return transitionToState(idleState);
            }

            liftMotor.setPower(pidOutput);

            return this;
        }
    }

    class LiftGoMediumJunctionState extends LiftClosedLoopState {

        LiftGoMediumJunctionState(Telemetry telemetry) {
            super("Lift-auto-med", telemetry, TimeUnit.SECONDS.toMillis(AUTO_STALL_TIMEOUT_SECONDS)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isTimedOut() && limitOverrideButton != null && !limitOverrideButton.isPressed()) {
                stopLift();
                Log.e(LOG_TAG, "Timed out while going to medium junction");

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

                setupPidController(K_P_MEDIUM_JUNCTION);

                //pidController.setOutputRange(-1, 1); // fix bouncing while descending
                pidController.setAbsoluteSetPoint(true); // MM
                pidController.setTarget(LIFT_MEDIUM_JUNCTION_ENCODER_POS,
                        liftMotor.getCurrentPosition());

                initialized = true;
            }

            // closed loop control based on motor encoders

            double pidOutput = pidController.getOutput(liftMotor.getCurrentPosition());

            if (pidController.isOnTarget()) {
                Log.d(LOG_TAG, "Lift reached medium target");

                stopLift();

                return transitionToState(idleState);
            }


            liftMotor.setPower(pidOutput);

            return this;
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
            } else if (fromButtonState != null && limitOverrideButton.isPressed()) {
                if (fromButtonState != this) {
                    Log.d(LOG_TAG, getName() + " - limits overridden - transitioning to " + fromButtonState.getName());

                    liftMotor.setPower(0);

                    return fromButtonState;
                }
            }

            // FIXME: Let's start with feed-forward only, and see if we really need a PID
            // to hold position

            liftMotor.setPower(ANTIGRAVITY_FEED_FORWARD);

            return this;
        }
    }


    class LiftAtLowerLimitState extends LiftBaseState {

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
            } else if (fromButtonState != null && limitOverrideButton.isPressed()) {
                if (fromButtonState != this) {
                    Log.d(LOG_TAG, getName() + " - limits overridden - transitioning to " + fromButtonState.getName());

                    return fromButtonState;
                }
            }

            return this;
        }
    }

    class LiftDownCommandState extends LiftBaseState {

        LiftDownCommandState(Telemetry telemetry) {
            super("Lift-cmd-dn", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            if (liftThrottleIsUp()) {
                    return transitionToState(upCommandState);
            } else if (liftUpperLimitButton.getRise()) {
                    return transitionToState(goUpperLimitState);
            } else if (liftLowerLimitButton.getRise()){
                    return transitionToState(goLowerLimitState);
            } else if (liftGoSmallButton.getRise()) {
                return transitionToState(goSmallJunctionState);
            } else if (liftGoMediumButton.getRise()) {
                return transitionToState(goMediumJunctionState);
            } else if (liftEmergencyStopButton.getRise()) {
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

    private boolean liftThrottleIsUp() {
        return liftThrottle.getPosition() < 0;
    }

    class LiftUpCommandState extends LiftBaseState {

        LiftUpCommandState(Telemetry telemetry) {
            super("Lift-cmd-up", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            if (liftThrottleIsDown()) {
                return transitionToState(downCommandState);
            } else if (liftUpperLimitButton.getRise()) {
                return transitionToState(goUpperLimitState);
            } else if (liftLowerLimitButton.getRise()){
                return transitionToState(goLowerLimitState);
            } else if (liftGoSmallButton.getRise()) {
                return transitionToState(goSmallJunctionState);
            } else if (liftGoMediumButton.getRise()) {
                return transitionToState(goMediumJunctionState);
            } else if (liftEmergencyStopButton.getRise()) {
                stopLift();

                return transitionToState(idleState);
            }

            if (limitSwitchIsOn(upperLiftLimit)) {
                stopLift();

                return transitionToState(atUpperLimitState);
            }

            if (!liftThrottleIsUp() /* FiXME: !liftCommandDownButton.isPressed()*/) {
                Log.d(LOG_TAG, "Lift - up command button released");
                stopLift();

                return transitionToState(idleState);
            }

            liftUp();

            return this;
        }
    }

    private static boolean limitSwitchIsOn(final DigitalChannel limitSwitch) {
        return limitSwitch != null && !limitSwitch.getState(); // Limit switches false -> on
    }
}
