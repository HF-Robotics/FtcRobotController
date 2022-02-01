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

package com.hfrobots.tnt.season2122;

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.ToggledButton;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import com.google.common.base.Ticker;
import com.google.common.collect.Sets;
import com.hfrobots.tnt.corelib.state.ReadyCheckable;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;
import java.util.concurrent.TimeUnit;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.NonNull;
import lombok.Setter;

/**
 * The class that makes the mechanism that picks up and delivers freight, work.
 */
public class FreightManipulator implements PeriodicTask {
    public static final int ARM_HIGH_POSITION = 436;
    // (1) We need to add the motors, servos and sensors this mechanism will use first, they go
    // in this location in the file. The mechanism requirements document can be consulted to
    // figure out what these are.

    final DcMotorEx armMotor;

    private final DigitalChannel lowPositionLimit;

    public static final double LEFT_FULL_POSITION = 0.0D;

    public static final double RIGHT_FULL_POSITION = 1.0D;

    public static final double CENTER_POSITION = 0.5D;

    public static final double LEFT_GRIPPER_OPEN_POSITION = 0.62;

    public static final double LEFT_GRIPPER_CLOSED_POSITION = 0.16;

    public static final double RIGHT_GRIPPER_OPEN_POSITION = 0.43;

    public static final double RIGHT_GRIPPER_CLOSED_POSITION = 0.881;

    public static final double UP_SPEED_DIVISIOR = 1.5;

    public static final double DOWN_SPEED_DIVISIOR = 6;

    public static final int ARM_SAFE_POSITION_DIFFERENCE = 200;

    public static final int ARM_LIMIT_POSITION_DIFFERENCE = 600;

    @Getter(AccessLevel.PACKAGE)
    @VisibleForTesting
    private int armMotorStartingPosition;

    private StateMachine stateMachine;

    private final Set<ReadyCheckable> readyCheckables = Sets.newHashSet();

    // private final Servo leftGripperServo;

    // private final Servo rightGripperServo;

    @Setter
    private OnOffButton intakeButton;

    @Setter
    private OnOffButton outtakeButton;

    private final CRServo intakeServo;

    @Setter
    private OnOffButton unsafeButton;

    @Setter
    private RangeInput armThrottle;

    @Setter
    private DebouncedButton toHubLevelOneButton;

    @Setter
    private DebouncedButton toHubLevelTwoButton;

    @Setter
    private DebouncedButton toHubLevelThreeButton;

    @Setter
    private ToggledButton gripperToggleButton;

    @Setter
    private boolean initFromTeleOp = false;

    private boolean notSafeToAutoMoveArm = false;

    private final Telemetry telemetry;

    private final Ticker ticker;

    public FreightManipulator(@NonNull final HardwareMap hardwareMap,
                              @NonNull final Telemetry telemetry,
                              @NonNull final Ticker ticker) {
        this.telemetry = telemetry;
        this.ticker = ticker;

        // (2) Here is where we setup the items in (1), by finding them in the HardwareMap
        //
        // If this mechanism will have its own automation through a state machine,
        // this would also be setup here.

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorStartingPosition = armMotor.getCurrentPosition();

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftGripperServo = hardwareMap.get(Servo.class, "leftGripperServo");
        //rightGripperServo = hardwareMap.get(Servo.class, "rightGripperServo");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        lowPositionLimit = hardwareMap.get(DigitalChannel.class, "lowPositionLimit");

        stateMachine = new StateMachine(telemetry);
        setupStateMachine();
    }

    // (3) Here, we define what the mechanism does, by adding methods. These methods contain
    // instructions for what to do with the items listed in step (1).

    public void moveArmUp(double speed) {
        final int currentPosition = armMotor.getCurrentPosition();

        boolean unsafePressed = unsafeButton != null && unsafeButton.isPressed();

        if (!unsafePressed) {
            if (currentPosition > armMotorStartingPosition + ARM_LIMIT_POSITION_DIFFERENCE) {

                stopArm();

                return;
            }
        }

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(speed / UP_SPEED_DIVISIOR);
    }

    public void moveArmDown(double speed) {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(-speed / DOWN_SPEED_DIVISIOR);
    }

    public void stopArm() {
        // Feed Forward
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(.25);
    }

    public void moveArmToLowGoal() {
        try {
            armMotor.setTargetPosition(armMotorStartingPosition + 150);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);
        } catch (TargetPositionNotSetException error) {
            Log.w(LOG_TAG, "Lost target position, disabling auto arm movement");
            notSafeToAutoMoveArm = true;
        }
    }

    public void moveArmToMiddleGoal() {
        try {
            armMotor.setTargetPosition(armMotorStartingPosition + 286);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);
        } catch (TargetPositionNotSetException error) {
            Log.w(LOG_TAG, "Lost target position, disabling auto arm movement");
            notSafeToAutoMoveArm = true;
        }
    }

    public void moveArmToTopGoal() {
        try {
            armMotor.setTargetPosition(armMotorStartingPosition + ARM_HIGH_POSITION);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);
        } catch (TargetPositionNotSetException error) {
            Log.w(LOG_TAG, "Lost target position, disabling auto arm movement");
            notSafeToAutoMoveArm = true;
        }
    }

    public void moveToSafePosition() {
        if (notSafeToAutoMoveArm) {
            telemetry.addLine("Not Moving to Safe Position");
            return;
        }

        if (intakeButton.isPressed()) {
            return;
        }

        if (unsafeButton != null && unsafeButton.isPressed()) {
            return;
        }

        final int currentPosition = armMotor.getCurrentPosition();

        if (currentPosition > armMotorStartingPosition + ARM_SAFE_POSITION_DIFFERENCE) {
            return; // already safe!
        }

        try {
            armMotor.setTargetPosition(armMotorStartingPosition + ARM_SAFE_POSITION_DIFFERENCE);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);
        } catch (TargetPositionNotSetException error) {
            Log.w(LOG_TAG, "Lost target position, disabling auto arm movement");
            notSafeToAutoMoveArm = true;
        }
    }

    public void openGripper() {
        //leftGripperServo.setPosition(LEFT_GRIPPER_OPEN_POSITION);
        //rightGripperServo.setPosition(RIGHT_GRIPPER_OPEN_POSITION);
    }

    public void closeGripper() {
        //leftGripperServo.setPosition(LEFT_GRIPPER_CLOSED_POSITION);
        //rightGripperServo.setPosition(RIGHT_GRIPPER_CLOSED_POSITION);
    }

    // (4) If this component needs to take action during every loop of tele-op or auto,
    // implement the PeriodicTask interface and its required methods, and write the
    // code that performs those actions in the periodicTask() method.

    @Override
    public void periodicTask() {
        // run the state machine
        stateMachine.doOneStateLoop();
    }

    protected String getCurrentStateName() {
        return stateMachine.getCurrentStateName();
    }

    // (5) If this mechanism will have automation using a state machine, we describe the
    // states as new classes here, and set them up in (2).

    private void setupStateMachine() {
        ToLowestPositionState toLowestPositionState = new ToLowestPositionState(ticker);

        IdleState idleState = new IdleState();
        toLowestPositionState.setIdleState(idleState);

        OpenLoopLiftLoweringState openLoopLiftLoweringState = new OpenLoopLiftLoweringState();
        OpenLoopLiftRisingState openLoopLiftRisingState = new OpenLoopLiftRisingState();
        openLoopLiftRisingState.setIdleState(idleState);
        idleState.setOpenLoopLiftRisingState(openLoopLiftRisingState);

        openLoopLiftLoweringState.setIdleState(idleState);
        idleState.setOpenLoopLiftLoweringState(openLoopLiftLoweringState);

        ClosedLoopArmMoveState toHubLevelOneState =
                new ClosedLoopArmMoveState("Lift - CL - One", 150);
        toHubLevelOneState.setIdleState(idleState);
        idleState.setToHubLevelOneState(toHubLevelOneState);

        ClosedLoopArmMoveState toHubLevelTwoState =
                new ClosedLoopArmMoveState("Lift - CL - Two", 286);
        toHubLevelTwoState.setIdleState(idleState);
        idleState.setToHubLevelTwoState(toHubLevelTwoState);

        ClosedLoopArmMoveState toHubLevelThreeState =
                new ClosedLoopArmMoveState("Lift - CL - Three", ARM_HIGH_POSITION);
        toHubLevelThreeState.setIdleState(idleState);
        idleState.setToHubLevelThreeState(toHubLevelThreeState);

        // Assert that all circular dependencies are set for all states
        for (ReadyCheckable toCheck : readyCheckables) {
            toCheck.checkReady();
        }

        stateMachine.setFirstState(toLowestPositionState);
    }

    class IdleState extends State implements ReadyCheckable {
        @Setter
        ClosedLoopArmMoveState toHubLevelOneState;

        @Setter
        ClosedLoopArmMoveState toHubLevelTwoState;

        @Setter
        ClosedLoopArmMoveState toHubLevelThreeState;

        @Setter
        OpenLoopLiftLoweringState openLoopLiftLoweringState;

        @Setter
        OpenLoopLiftRisingState openLoopLiftRisingState;

        protected IdleState() {
            super("Lift Idle", FreightManipulator.this.telemetry);
            readyCheckables.add(this);
        }

        @Override
        public State doStuffAndGetNextState() {

            stopArm();

            final boolean atLowerLimit = limitSwitchOn(lowPositionLimit);

            if (atLowerLimit) {
                int currentPosition = armMotor.getCurrentPosition();

                if (armMotorStartingPosition != currentPosition) {
                    armMotorStartingPosition = currentPosition;
                    Log.d(LOG_TAG, "Reset arm motor starting position to: " + armMotorStartingPosition);
                }
            }

            if (intakeButton != null && intakeButton.isPressed()) {
                spinIntake();
            } else if (outtakeButton != null && outtakeButton.isPressed()) {
                spinOuttake();
            } else {
                stopIntake();
            }


            if (armThrottle.getPosition() < 0) {
                return openLoopLiftRisingState;
            } else if (armThrottle.getPosition() > 0 && (!unsafeButton.isPressed() && !atLowerLimit)) {
                return openLoopLiftLoweringState;
            } else if (toHubLevelOneButton.getRise()) {
                return toHubLevelOneState;
            } else if (toHubLevelTwoButton.getRise()) {
                return toHubLevelTwoState;
            } else if (toHubLevelThreeButton.getRise()) {
                return toHubLevelThreeState;
            }

            return this;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(toHubLevelOneState);
            Preconditions.checkNotNull(toHubLevelTwoState);
            Preconditions.checkNotNull(toHubLevelThreeState);
            Preconditions.checkNotNull(openLoopLiftLoweringState);
            Preconditions.checkNotNull(openLoopLiftRisingState);
        }
    }

    public void stopIntake() {
        intakeServo.setPower(0);
    }

    public void spinOuttake() {
        intakeServo.setPower(-1);
    }

    public void spinIntake() {
        intakeServo.setPower(1);
    }

    class OpenLoopLiftLoweringState extends State implements ReadyCheckable {
        @Setter
        private IdleState idleState;

        protected OpenLoopLiftLoweringState() {
            super("Lift - OL - Lowering", FreightManipulator.this.telemetry);
            readyCheckables.add(this);
        }

        @Override
        public State doStuffAndGetNextState() {
            final boolean atLowerLimit = limitSwitchOn(lowPositionLimit);

            if (atLowerLimit && !unsafeButton.isPressed()) {
                return idleState;
            }

            final float armThrottlePosition = armThrottle.getPosition();

            if (armThrottlePosition > 0) {
                moveArmDown(Math.abs(armThrottlePosition));

                return this;
            }

            return idleState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(idleState);
        }
    }

    class OpenLoopLiftRisingState extends State implements ReadyCheckable {
        @Setter
        private IdleState idleState;

        protected OpenLoopLiftRisingState() {
            super("Lift - OL - Rising", FreightManipulator.this.telemetry);
            readyCheckables.add(this);
        }

        @Override
        public State doStuffAndGetNextState() {
            final float armThrottlePosition = armThrottle.getPosition();

            if (armThrottlePosition < 0) {
                moveArmUp(Math.abs(armThrottlePosition));

                return this;
            }

            return idleState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(idleState);
        }

    }

    class ClosedLoopArmMoveState extends StopwatchTimeoutSafetyState implements ReadyCheckable {
        private final int targetPosition;

        private boolean headingToPosition = false;

        @Setter
        private IdleState idleState;

        protected ClosedLoopArmMoveState(@NonNull final String name,
                                         final int targetPosition) {
            super(name, FreightManipulator.this.telemetry, ticker, 4000);
            this.targetPosition = targetPosition;
            readyCheckables.add(this);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (notSafeToAutoMoveArm) {
                resetTimer();

                return idleState;
            }

            if (isTimedOut()) {
                resetTimer();

                return idleState;
            }

            if (unsafeButton.isPressed()) {
                resetTimer();

                return idleState;
            }

            final float armThrottlePosition = armThrottle.getPosition();

            if (armThrottlePosition != 0) {
                headingToPosition = false;

                resetTimer();

                return idleState;
            }

            if (!headingToPosition) {
                try {
                    armMotor.setTargetPosition(armMotorStartingPosition + targetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.5);
                    headingToPosition = true;

                    return this;
                } catch (TargetPositionNotSetException error) {
                    Log.w(LOG_TAG, "Lost target position, disabling auto arm movement");
                    notSafeToAutoMoveArm = true;

                    return idleState; // be safe!
                }
            } else {
                // This is the test for "until it gets there" in our diagram...
                if (!armMotor.isBusy()) {
                    headingToPosition = false;

                    resetTimer();

                    return idleState;
                }
            }

            return this;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(idleState);
        }
    }

    class ToLowestPositionState extends StopwatchTimeoutSafetyState implements ReadyCheckable {
        @Setter
        private IdleState idleState;

        protected ToLowestPositionState(@NonNull final Ticker ticker) {
            super("To Lowest Pos", FreightManipulator.this.telemetry, ticker, TimeUnit.SECONDS.toMillis(2));
            readyCheckables.add(this);
        }

        @Override
        public State doStuffAndGetNextState() {
            final float armThrottlePosition = armThrottle.getPosition();

            if (armThrottlePosition != 0) {
                resetTimer();

                return idleState;
            }

            if (limitSwitchOn(lowPositionLimit)) {
                resetTimer();

                return idleState;
            }

            if (isTimedOut()) {
                resetTimer();

                return idleState;
            }

            moveArmDown(.05);

            return this;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(idleState);
        }
    }

    private static boolean limitSwitchOn(DigitalChannel channel) {
        return !channel.getState();
    }
}
