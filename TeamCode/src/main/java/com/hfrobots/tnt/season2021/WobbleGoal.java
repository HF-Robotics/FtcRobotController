/*
 Copyright (c) 2021 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season2021;

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.ToggledButton;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import com.google.common.base.Ticker;
import com.google.common.collect.Lists;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;

import lombok.Builder;
import lombok.NonNull;
import lombok.Setter;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class WobbleGoal {

    private DcMotorEx shoulderMotor;

    private Servo gripperServo;

    private DigitalChannel placeLimitSwitch;

    private DigitalChannel stowLimitSwitch;

    @Setter
    private RangeInput shoulderThrottle;

    @Setter
    private ToggledButton gripperButton;

    @Setter
    private OnOffButton unsafe;

    private final static int PLACE_POS_ENCODER_COUNT = 0; //FIXME this is not correct

    public final static double OPEN_GRIPPER_POS = .25;

    public final static double CLOSED_GRIPPER_POS = 0;

    public static final float TOWARDS_STOW_POWER_MAGNITUDE = -1;

    public static final float TOWARDS_PLACE_POWER_MAGNITUDE = 1;

    private State currentState;

    private List<ReadyCheckable> readyCheckables = Lists.newArrayList();

    private AutoStowState autoStowState;

    private AutoPlaceState autoPlaceState;

    @Builder
    private WobbleGoal(HardwareMap hardwareMap, Telemetry telemetry, Ticker ticker) {
        shoulderMotor = hardwareMap.get(DcMotorEx.class, "shoulderMotor");

        gripperServo = hardwareMap.get(Servo.class, "gripperServo");

        placeLimitSwitch = hardwareMap.get(DigitalChannel.class, "placeLimitSwitch");

        stowLimitSwitch = hardwareMap.get(DigitalChannel.class, "stowLimitSwitch");

        PlaceState placeState = new PlaceState(telemetry);

        MotionState motionState = new MotionState(telemetry);

        StowState stowState = new StowState(telemetry);

        placeState.setMotionState(motionState);
        stowState.setMotionState(motionState);

        motionState.setPlaceState(placeState);
        motionState.setStowState(stowState);

        autoPlaceState = new AutoPlaceState(telemetry, ticker);
        autoPlaceState.setNextState(placeState);

        autoStowState = new AutoStowState(telemetry,ticker);
        autoStowState.setNextState(stowState);

        // because of circular dependencies during construction, we need to post-check
        // that all of the transitions have been setup correctly

        for (ReadyCheckable checkMe : readyCheckables) {
            checkMe.checkReady();
        }

        // FIXME: Why does this work? (or more - how do we make sure this works, to start this way?)
        currentState = motionState;
    }

    public void gotoPlaceState() {
        currentState = autoPlaceState;
    }

    public void gotoStowState() {
        currentState = autoStowState;
    }

    public void periodicTask() {
        if (currentState != null) {
            State nextState = currentState.doStuffAndGetNextState();

            if (nextState != currentState) {
                // We've changed states alert the driving team, log for post-match analysis
                //telemetry.addData("00-State", "From %s to %s", currentState, nextState);
                Log.d(LOG_TAG, String.format("Wobble goal transition from %s to %s", currentState.getClass()
                        + "(" + currentState.getName() + ")", nextState.getClass() + "(" + nextState.getName() + ")"));
            }

            currentState = nextState;
        } else {
            Log.e(LOG_TAG, "No state machine setup!");
        }
    }

    public void openGripper() {
        gripperServo.setPosition(OPEN_GRIPPER_POS);
    }

    public void closeGripper() {
        gripperServo.setPosition(CLOSED_GRIPPER_POS);
    }

    @VisibleForTesting
    State getCurrentState() {
        return currentState;
    }

    // Handles the things we can do while placing the wobble goal, namely
    // grip/un-grip and move the arm away from the placed state
    class PlaceState extends NotDebuggableState {
        @Setter
        private MotionState motionState;

        protected PlaceState(Telemetry telemetry) {
            super("PlaceState", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {

            handleGripperServo();

            if (requestTowardsStow()) {
                return motionState;
            }

            return this;
        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(motionState);
        }
    }

    private void handleGripperServo() {
        if (gripperButton.isToggledTrue()) {
            gripperServo.setPosition(OPEN_GRIPPER_POS);
        } else {
            gripperServo.setPosition(CLOSED_GRIPPER_POS);
        }
    }

    // Handles the things we can do while stowed (essentially move away from stowed)
    class StowState extends NotDebuggableState {

        @Setter
        private MotionState motionState;

        protected StowState(Telemetry telemetry) {
            super("StowState", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (requestTowardsPlace()) {
                return motionState;
            }

            if (unsafe.isPressed()){
                handleGripperServo();
            }

            return this;
        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(motionState);
        }
    }

    abstract class AutoMotionState extends StopwatchTimeoutSafetyState implements ReadyCheckable {
        final DigitalChannel limitSwitch;

        final float motorPower;

        protected AutoMotionState(final String name, final Telemetry telemetry, final Ticker ticker,
                                  final DigitalChannel limitSwitch, final float motorPower) {
            super(name, telemetry, ticker, TimeUnit.SECONDS.toMillis(8));
            this.limitSwitch = limitSwitch;
            this.motorPower = motorPower;
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isTimedOut() || limitSwitchOn(limitSwitch)) {
                if (isTimedOut())  {
                    Log.w(LOG_TAG, name + " timed out waiting for limit switch");
                }

                shoulderMotor.setPower(0);

                resetTimer();

                return nextState;
            }

            shoulderMotor.setPower(motorPower);

            return this;
        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(nextState);
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }

    class AutoPlaceState extends AutoMotionState{
        protected AutoPlaceState(Telemetry telemetry, Ticker ticker) {
            super("AutoPlaceState", telemetry, ticker,
            placeLimitSwitch, TOWARDS_PLACE_POWER_MAGNITUDE);
        }
    }

    class AutoStowState extends AutoMotionState{
        protected AutoStowState(Telemetry telemetry, Ticker ticker) {
            super("AutoStowState", telemetry, ticker,
                    stowLimitSwitch, TOWARDS_STOW_POWER_MAGNITUDE);
        }
    }


    // Handles the motion when not placing, and not stowed. Needs to detect when
    // when the arm reaches either of those positions and do change to the
    // correct state (place or stow).
    class MotionState extends NotDebuggableState {

        @Setter
        private PlaceState placeState;

        @Setter
        private StowState stowState;

        protected MotionState(Telemetry telemetry) {
            super("MotionState", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (requestTowardsStow()) {
                // we are heading towards stowed position, are we there?
                if (!unsafe.isPressed() && limitSwitchOn(stowLimitSwitch)) {
                    shoulderMotor.setPower(0);

                    return stowState;
                }

                // otherwise, adjust power to the shoulder motor
                setShoulderMotorPower(TOWARDS_STOW_POWER_MAGNITUDE);

                return this;
            } else if (requestTowardsPlace()) {
                // we are heading towards the placed position, are we there?
                if (!unsafe.isPressed() && limitSwitchOn(placeLimitSwitch)) {
                    shoulderMotor.setPower(0);

                    return placeState;
                }


                // otherwise adjust power to the shoulder motor
                setShoulderMotorPower(TOWARDS_PLACE_POWER_MAGNITUDE);

                return this;
            } else {
                // FIXME: What case is this? (not asking for forward or backward motion)
                shoulderMotor.setPower(0);
            }

            if (unsafe.isPressed()) {
                handleGripperServo();
            }


            // TODO: Doubtful (because of gearing), but we may need feedforward to hold position?

            return this;
        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(placeState);
            Preconditions.checkNotNull(stowState);
        }
    }

    private void setShoulderMotorPower(double powerMagnitude) {
        float absoluteThrottle = Math.abs(shoulderThrottle.getPosition());
        shoulderMotor.setPower(powerMagnitude * absoluteThrottle);
    }

    private boolean requestTowardsPlace() {
        return shoulderThrottle.getPosition() < 0;
    }

    private boolean requestTowardsStow() {
        return shoulderThrottle.getPosition() > 0;
    }

    //
    // Just here to remove some boiler plate code that's not really used by our
    // implementation
    //

    abstract class NotDebuggableState extends State implements ReadyCheckable {

        protected NotDebuggableState(@NonNull String name, Telemetry telemetry) {
            super(name, telemetry);
            readyCheckables.add(this);
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }

    private static boolean limitSwitchOn(DigitalChannel channel) {
        return !channel.getState();
    }
}
