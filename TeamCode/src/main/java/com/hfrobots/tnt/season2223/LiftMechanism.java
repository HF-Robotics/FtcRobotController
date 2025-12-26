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
import com.ftc9929.corelib.state.RunnableState;
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
import lombok.NonNull;
import lombok.Setter;

public class LiftMechanism {

    private LiftIdleState idleState;
    private AutoGrabConeCloseServoState grabConeCloseServoState;

    static abstract class MotorSpecificConstants {
        double getAntigravityFeedForward() {
            return .24;
        }

        double getLiftPowerLevel() {
            return 1;
        }

        double getKPUpperLimit() {
            return .008;
        }

        double getKPLowerLimit() {
            return .0008;
        }

        double getKPSmallJunction() {
            return .004;
        }

        double getKPMediumJunction() {
            return .004;
        }

        double getOpenLoopDownPowerRatio() {
            return .02;
        }

        abstract int getLiftPlacePreloadSmallJunctionEncoderPos();

        abstract int getLiftSmallJunctionEncoderPos();

        abstract int getLiftMediumJunctionEncoderPos();

        abstract int getUpperLimitEncoderPos();

        abstract int getLowerLimitEncoderPos();
    }

    @SuppressWarnings("unused")
    private static class MotorConstants_19_1 extends MotorSpecificConstants {
        private static final int LIFT_PLACE_PRELOAD_SMALL_JUNCTION_ENCODER_POS = 400;

        private static final int LIFT_SMALL_JUNCTION_ENCODER_POS = 473;

        private static final int LIFT_MEDIUM_JUNCTION_ENCODER_POS = 857;

        private static final int LIFT_UPPER_LIMIT_ENCODER_POS = 1224;

        private static final int LIFT_LOWER_LIMIT_ENCODER_POS = 0;

        @Override
        int getLiftPlacePreloadSmallJunctionEncoderPos() {
            return LIFT_PLACE_PRELOAD_SMALL_JUNCTION_ENCODER_POS;
        }

        @Override
        int getLiftSmallJunctionEncoderPos() {
            return LIFT_SMALL_JUNCTION_ENCODER_POS;
        }

        @Override
        int getLiftMediumJunctionEncoderPos() {
            return LIFT_MEDIUM_JUNCTION_ENCODER_POS;
        }

        @Override
        int getUpperLimitEncoderPos() {
            return LIFT_UPPER_LIMIT_ENCODER_POS;
        }

        @Override
        int getLowerLimitEncoderPos() {
            return LIFT_LOWER_LIMIT_ENCODER_POS;
        }
    }

    private static class MotorConstants_26_1 extends MotorSpecificConstants {
        private static final int LIFT_PLACE_PRELOAD_SMALL_JUNCTION_ENCODER_POS = 580;

        private static final int LIFT_SMALL_JUNCTION_ENCODER_POS = 850;

        private static final int LIFT_MEDIUM_JUNCTION_ENCODER_POS = 1340;

        private static final int LIFT_UPPER_LIMIT_ENCODER_POS = 1800;

        private static final int LIFT_LOWER_LIMIT_ENCODER_POS = 0;

        @Override
        int getLiftPlacePreloadSmallJunctionEncoderPos() {
            return LIFT_PLACE_PRELOAD_SMALL_JUNCTION_ENCODER_POS;
        }

        @Override
        int getLiftSmallJunctionEncoderPos() {
            return LIFT_SMALL_JUNCTION_ENCODER_POS;
        }

        @Override
        int getLiftMediumJunctionEncoderPos() {
            return LIFT_MEDIUM_JUNCTION_ENCODER_POS;
        }

        @Override
        int getUpperLimitEncoderPos() {
            return LIFT_UPPER_LIMIT_ENCODER_POS;
        }

        @Override
        int getLowerLimitEncoderPos() {
            return LIFT_LOWER_LIMIT_ENCODER_POS;
        }
    }

    // Allows us to fall back to various known, working constants
    // if we have to swap out a motor for a different type
    private final static MotorSpecificConstants MOTOR_SPECIFIC_CONSTANTS
            = new MotorConstants_26_1();

    private static final double LIFT_POWER_LEVEL = MOTOR_SPECIFIC_CONSTANTS
            .getLiftPowerLevel();

    private static final double K_P_UPPER_LIMIT = MOTOR_SPECIFIC_CONSTANTS
            .getKPUpperLimit();

    private static final double K_P_LOWER_LIMIT = MOTOR_SPECIFIC_CONSTANTS
            .getKPLowerLimit();

    private static final double K_P_SMALL_JUNCTION = MOTOR_SPECIFIC_CONSTANTS
            .getKPSmallJunction();

    private static final double K_P_MEDIUM_JUNCTION = MOTOR_SPECIFIC_CONSTANTS
            .getKPMediumJunction();

    private static final double ANTIGRAVITY_FEED_FORWARD =
            MOTOR_SPECIFIC_CONSTANTS.getAntigravityFeedForward();

    private static final int LIFT_PLACE_PRELOAD_SMALL_JUNCTION_ENCODER_POS =
            MOTOR_SPECIFIC_CONSTANTS.getLiftPlacePreloadSmallJunctionEncoderPos();

    private static final int LIFT_SMALL_JUNCTION_ENCODER_POS =
            MOTOR_SPECIFIC_CONSTANTS.getLiftSmallJunctionEncoderPos();

    private static final int LIFT_MEDIUM_JUNCTION_ENCODER_POS =
            MOTOR_SPECIFIC_CONSTANTS.getLiftMediumJunctionEncoderPos();

    private static final int LIFT_UPPER_LIMIT_ENCODER_POS =
            MOTOR_SPECIFIC_CONSTANTS.getUpperLimitEncoderPos();

    private static final int LIFT_LOWER_LIMIT_ENCODER_POS =
            MOTOR_SPECIFIC_CONSTANTS.getLowerLimitEncoderPos();

    private static final int AUTO_STALL_TIMEOUT_SECONDS = 8;

    private static final double OPEN_LOOP_DOWN_POWER_RATIO =
            MOTOR_SPECIFIC_CONSTANTS.getOpenLoopDownPowerRatio();

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
    private DebouncedButton autoGrabConeButton;

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

    private LiftGoScorePreloadConeState goScorePreloadConeState;

    private void setupStateMachine(Telemetry telemetry) {
        // TODO - Go through this with team
        LiftUpLittleBitState liftUpLittleBitState = new LiftUpLittleBitState(telemetry);
        grabConeCloseServoState = new AutoGrabConeCloseServoState(liftUpLittleBitState, telemetry);

        LiftGoUpperLimitState goUpperLimitState = new LiftGoUpperLimitState(telemetry);

        LiftGoLowerLimitState goLowerLimitState = new LiftGoLowerLimitState(telemetry);

        goScorePreloadConeState = new LiftGoScorePreloadConeState(telemetry);

        LiftGoSmallJunctionState goSmallJunctionState = new LiftGoSmallJunctionState(telemetry);

        LiftGoMediumJunctionState goMediumJunctionState = new LiftGoMediumJunctionState(telemetry);

        LiftDownCommandState downCommandState = new LiftDownCommandState(telemetry);

        LiftUpCommandState upCommandState = new LiftUpCommandState(telemetry);

        LiftAtUpperLimitState atUpperLimitState = new LiftAtUpperLimitState(telemetry);

        LiftAtLowerLimitState atLowerLimitState = new LiftAtLowerLimitState(telemetry);

        idleState = new LiftIdleState(telemetry);

        grabConeCloseServoState.setAllTransitionToStates(grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        liftUpLittleBitState.setAllTransitionToStates(grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        goScorePreloadConeState.setAllTransitionToStates(grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        goUpperLimitState.setAllTransitionToStates(grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
         upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        goSmallJunctionState.setAllTransitionToStates(grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        goMediumJunctionState.setAllTransitionToStates(grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        goLowerLimitState.setAllTransitionToStates(grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        downCommandState.setAllTransitionToStates(grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        upCommandState.setAllTransitionToStates (grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        atUpperLimitState.setAllTransitionToStates(grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        atLowerLimitState.setAllTransitionToStates(grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        idleState.setAllTransitionToStates(grabConeCloseServoState, goUpperLimitState, goSmallJunctionState, goMediumJunctionState, goLowerLimitState, downCommandState,
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

    protected void liftToScorePreloadPosition() {
        currentState = goScorePreloadConeState;
    }

    protected void grabAndLiftCone() {
        currentState = grabConeCloseServoState;
    }

    protected boolean isLiftAboveSlowHeight() {
        int currentPos = liftMotor.getCurrentPosition();

        if (currentPos >= LIFT_MEDIUM_JUNCTION_ENCODER_POS - 120) {
           return true;
        } else {
            return false;
        }
    }

    public void periodicTask() {
        String currentStateName = getCurrentStateName();

        telemetry.addData("SM: ", "e: " + currentStateName + " b: " +
                (limitOverrideButton.isPressed() ? "!" : ""));

        State nextState = currentState.doStuffAndGetNextState();

        if (nextState == null) {
            nextState = idleState;
        }

        if (nextState != currentState) {
            // We've changed states alert the driving team, log for post-match analysis
            telemetry.addData("00-State", "From %s to %s", currentState, nextState);
            Log.d(LOG_TAG, String.format("State transition from %s to %s", currentState.getClass()
                    + "(" + currentState.getName() + ")", nextState.getClass() + "(" + nextState.getName() + ")"));
        }

        currentState = nextState;
    }

    protected String getCurrentStateName() {
        String currentStateName = currentState.getName();

        if (currentStateName != null) {
            currentStateName = currentStateName.replace("Lift-", "");
        } else {
            currentStateName = "Unk";
        }
        return currentStateName;
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

        AutoGrabConeCloseServoState autoGrabConeCloseServoState;

        LiftBaseState(String name, Telemetry telemetry, long timeoutMillis) {
            super(name, telemetry, timeoutMillis);
        }

        protected void setAllTransitionToStates(
                final AutoGrabConeCloseServoState autoGrabConeCloseServoState,
                final LiftGoUpperLimitState goUpperLimitState,
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

            this.autoGrabConeCloseServoState = autoGrabConeCloseServoState;

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
            } else if (autoGrabConeButton != null && autoGrabConeButton.getRise()) {
                return autoGrabConeCloseServoState;
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

                stopLift();

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

    class LiftGoScorePreloadConeState extends LiftClosedLoopState {

        LiftGoScorePreloadConeState(Telemetry telemetry) {
            super("Lift-auto-preload-small", telemetry, TimeUnit.SECONDS.toMillis(AUTO_STALL_TIMEOUT_SECONDS)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isTimedOut() && limitOverrideButton != null && !limitOverrideButton.isPressed()) {
                stopLift();
                Log.e(LOG_TAG, "Timed out while going to small junction");

                return transitionToState(idleState);
            }

            if (!initialized) {
                Log.d(LOG_TAG, "Initializing PID for state " + getName());

                setupPidController(K_P_SMALL_JUNCTION);

                pidController.setOutputRange(-1, 1); // fix bouncing while descending
                pidController.setAbsoluteSetPoint(true); // MM

                pidController.setTarget(LIFT_PLACE_PRELOAD_SMALL_JUNCTION_ENCODER_POS,
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

    class AutoGrabConeCloseServoState extends LiftClosedLoopState {
        long servoClosedAtMillis;

        final long WAIT_SERVO_CLOSE_MS = 500;

        final LiftUpLittleBitState liftUpLittleBitState;

        AutoGrabConeCloseServoState(LiftUpLittleBitState liftUpLittleBitState, Telemetry telemetry) {
            super("Lift-auto-grab-closeServo", telemetry, TimeUnit.SECONDS.toMillis(AUTO_STALL_TIMEOUT_SECONDS)); // FIXME
            this.liftUpLittleBitState = liftUpLittleBitState;
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null) {
                return transitionToState(fromButtonState);
            }

            if (!initialized) {
                servoClosedAtMillis = System.currentTimeMillis();
                initialized = true;
            }


            gripper.close();
            // (2) Check if timer has expired - move to next state

            if (timerHasExpired()) {
                return transitionToState(liftUpLittleBitState);
            }

            return this;
        }

        private boolean timerHasExpired() {
            return System.currentTimeMillis() - servoClosedAtMillis > WAIT_SERVO_CLOSE_MS;
        }
    }

    class LiftUpLittleBitState extends LiftClosedLoopState {

        LiftUpLittleBitState(Telemetry telemetry) {
            super("Lift-up-a-bit", telemetry, TimeUnit.SECONDS.toMillis(AUTO_STALL_TIMEOUT_SECONDS)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isTimedOut() && limitOverrideButton != null && !limitOverrideButton.isPressed()) {
                stopLift();
                Log.e(LOG_TAG, "Timed out while lifting up a little bit");

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

                setupPidController(.04);

                pidController.setTargetTolerance(45);
                pidController.setOutputRange(-1, 1); // fix bouncing while descending
                pidController.setAbsoluteSetPoint(true);

                int upALittleBitPos = liftMotor.getCurrentPosition() + 120;
                pidController.setTarget(upALittleBitPos,
                        liftMotor.getCurrentPosition());

                initialized = true;
            }

            // closed loop control based on motor encoders

            double pidOutput = pidController.getOutput(liftMotor.getCurrentPosition());

            if (pidController.isOnTarget()) {
                Log.d(LOG_TAG, "Lift reached up a little bit target");

                stopLift();

                return transitionToState(idleState);
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
