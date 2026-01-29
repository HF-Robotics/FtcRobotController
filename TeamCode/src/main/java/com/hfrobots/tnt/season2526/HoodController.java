/*
 Copyright (c) 2026 The Tech Ninja Team (https://ftc9929.com)

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
import com.hfrobots.tnt.corelib.drive.PidController;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.hfrobots.tnt.season2324.Shared;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.Setter;

public class HoodController implements PeriodicTask {

    private final CRServo hoodAngleServo;

    private final DcMotorEx hoodAngleEncoder;

    private final DigitalChannel hoodLowerLimit;

    private final Telemetry telemetry;

    private final Ticker ticker;

    @Setter
    private RangeInput manualControl;

    private int currentHomePosition;

    private State currentState;

    private WheeledLauncher.TargetDistance currentHoodPosition = null;

    public HoodController(final HardwareMap hardwareMap, final Telemetry telemetry, Ticker ticker) {
        hoodAngleServo = hardwareMap.get(CRServo.class, "hoodAngleServo");
        hoodAngleEncoder = hardwareMap.get(DcMotorEx.class, "leftRearDriveMotor");
        hoodLowerLimit = hardwareMap.get(DigitalChannel.class, "hoodLowerLimit");
        this.telemetry = telemetry;
        this.ticker = ticker;
        this.currentHomePosition = hoodAngleEncoder.getCurrentPosition();

        setupStateMachine();
    }

    private State idleState;

    private GoToPositionState goCloseDistanceState;

    private GoToPositionState goMediumDistanceState;

    private GoToPositionState goFarDistanceState;

    private State goHomeState;

    private void setupStateMachine() {
        goHomeState = new GoHomeState(telemetry);
        idleState = new IdleState(telemetry);

        goCloseDistanceState = new GoToPositionState("Go-Close", telemetry, WheeledLauncher.CLOSE_HOOD_POSITION, WheeledLauncher.TargetDistance.CLOSE);

        goMediumDistanceState = new GoToPositionState("Go-Med", telemetry, WheeledLauncher.MEDIUM_HOOD_POSITION, WheeledLauncher.TargetDistance.MEDIUM);

        goFarDistanceState = new GoToPositionState("Go-Far", telemetry, WheeledLauncher.FAR_HOOD_POSITION, WheeledLauncher.TargetDistance.FAR);

        goCloseDistanceState.setNextState(idleState);
        goMediumDistanceState.setNextState(idleState);
        goFarDistanceState.setNextState(idleState);
        goHomeState.setNextState(idleState);

        currentState = idleState;
    }

    public void setDynamicPosition(final int position) {
        if (position < 0) {
            return;
        }

        State goDynamicDistanceState = new GoToPositionState("Go-Close", telemetry, position, WheeledLauncher.TargetDistance.CLOSE);
        goDynamicDistanceState.setNextState(idleState);
        currentState = goDynamicDistanceState;
    }

    public void setPosition(final WheeledLauncher.TargetDistance targetDistance) {
        if (currentHoodPosition != null && currentHoodPosition == targetDistance) {
            return;
        }

        switch (targetDistance) {
            case CLOSE:
                currentState = goCloseDistanceState;
                break;
            case MEDIUM:
                currentState = goMediumDistanceState;
                break;
            case FAR:
                currentState = goFarDistanceState;
                break;
        }
    }

    public void goHome() {
        currentState = goHomeState;
    }

    public boolean isIdle() {
        return currentState == idleState;
    }

    @Override
    public void periodicTask() {
        String currentStateName = getCurrentStateName();

        if (telemetry != null) {
            telemetry.addData("Hood", "ang: %d, st: %s",
                    getRelativeEncoderPosition(),
                    currentStateName);
        }

        doOneStateMachineLoop();
    }

    private void doOneStateMachineLoop() {
        Shared.withBetterErrorHandling(() -> {
            State nextState = currentState.doStuffAndGetNextState();

            if (nextState == null) {
                nextState = idleState;
            }

            if (nextState != currentState) {
                Log.d(LOG_TAG, String.format("Hood state transition from %s to %s", currentState.getClass()
                        + "(" + currentState.getName() + ")", nextState.getClass() + "(" + nextState.getName() + ")"));
            }

            currentState = nextState;
        });
    }

    protected String getCurrentStateName() {
        if (currentState == null) {
            return "Unk";
        }

        String currentStateName = currentState.getName();

        if (currentStateName == null) {
            currentStateName = "Unk";
        }

        return currentStateName;
    }

    class GoHomeState extends StopwatchTimeoutSafetyState {

        protected GoHomeState(Telemetry telemetry) {
            super("Hood homing", telemetry, ticker, 5_000);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isAtLowerLimit()) {
                Log.e(LOG_TAG, "Lower limit detected while homing hood, going idle");

                currentHomePosition = hoodAngleEncoder.getCurrentPosition();
                hoodAngleServo.setPower(0);

                resetToStart();

                return nextState;
            }

            if (isTimedOut()) {
                Log.e(LOG_TAG, "Timed out while homing hood, going idle");

                hoodAngleServo.setPower(0);

                resetToStart();

                return nextState;
            }

            hoodAngleServo.setPower(-1);

            return this;
        }
    }

    private boolean isAtLowerLimit() {
        return hoodLowerLimit.getState() == false;
    }

    class GoToPositionState extends StopwatchTimeoutSafetyState {
        private final PidController pidController;

        private final int targetPosition;

        private final WheeledLauncher.TargetDistance requestedTargetDistance;

        private boolean targetInitialized = false;

        private boolean requireSecondRun = false;

        protected GoToPositionState(final String name,
                                    final Telemetry telemetry,
                                    final int targetPosition, WheeledLauncher.TargetDistance requestedTargetDistance) {
            super(name, telemetry, ticker, 10_000);

            this.targetPosition = targetPosition;

            // FIXME: Encoder is 8000-ish per 1 full revolution
            //        Hood is 1/4 revolution at best
            //        What kP gives us a slow-down for the right
            //        amount of degrees (1/16 or 1/32 rotation?)
            pidController = PidController.builder().setInstanceName(name)
                    .setKp(.0018)
                    .setAllowOscillation(true)
                    .setTolerance(5)
                    .build();
            this.requestedTargetDistance = requestedTargetDistance;
            pidController.setAbsoluteSetPoint(true);
            pidController.setOutputRange(-1, 1);
        }

        @Override
        public void resetToStart() {
            super.resetToStart();
            pidController.reset();
            targetInitialized = false;
            requireSecondRun = false;
        }

        @Override
        public State doStuffAndGetNextState() {

            if (manualControl != null && manualControl.getPosition() != 0) {
                prepareToTransitionToNextState();
                currentHoodPosition = null;

                return idleState;
            }

            if (!targetInitialized) {
                pidController.setTarget(targetPosition, getRelativeEncoderPosition());

                final double initialError = pidController.getError();

                // Longer travels need a second run at the PID
                // for consistency's sake
                requireSecondRun = Math.abs(initialError) > 100;

                targetInitialized = true;
            }

            if (pidController.isOnTarget()) {
                Log.i(LOG_TAG, "Reached target, going idle");
                prepareToTransitionToNextState();

                currentHoodPosition = requestedTargetDistance;

                if (requireSecondRun) {
                    return this;
                }

                return nextState;
            }

            if (isTimedOut()) {
                Log.i(LOG_TAG, "Timed out before reaching target, going idle");
                prepareToTransitionToNextState();

                currentHoodPosition = null;

                if (requireSecondRun) {
                    return this;
                }

                return nextState;
            }

            double output = pidController.getOutput(getRelativeEncoderPosition());

            if (output < 0 && isAtLowerLimit()) {
                Log.e(LOG_TAG, "Huh! Reached lower limit, resetting and starting over");
                prepareToTransitionToNextState();
                currentHomePosition = hoodAngleEncoder.getCurrentPosition();

                return this;
            }

            // There's a periodic lower output limit where the mechanism gets stuck
            if (output > 0) {
                if (output < 0.053) {
                    output = 0.053;
                }
            } else if (output < 0) {
                if (output > -0.053) {
                    output = -0.053;
                }
            }

            hoodAngleServo.setPower(output);

            return this;
        }

        private void prepareToTransitionToNextState() {
            hoodAngleServo.setPower(0);

            resetToStart();
        }
    }

    private int getRelativeEncoderPosition() {
        return hoodAngleEncoder.getCurrentPosition() - currentHomePosition;
    }

    class IdleState extends State {
        protected IdleState(Telemetry telemetry) {
            super("Hood idle", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (manualControl != null) {
                final float position = manualControl.getPosition();

                if (position != 0) {
                    // do stuff based on the position
                    hoodAngleServo.setPower(position);
                    currentHoodPosition = null;
                } else {
                    hoodAngleServo.setPower(0);
                }
            }

            handleEncoderCoastAfterHoming();

            return this;
        }

        private void handleEncoderCoastAfterHoming() {
            if (hoodAngleServo.getPower() == 0) {
                if (getRelativeEncoderPosition() < 0 && !hoodLowerLimit.getState()) {
                    currentHomePosition = hoodAngleEncoder.getCurrentPosition();
                }
            }
        }

        @Override
        public void resetToStart() {

        }
    }
}
