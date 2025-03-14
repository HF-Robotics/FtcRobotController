/*
 Copyright (c) 2024 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.season2425;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.ParametricScaledRangeInput;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.RangeInputButton;
import com.hfrobots.tnt.corelib.task.PeriodicTask;

import lombok.Builder;

// FIXME: Copy this class into the season package, and rename it to include the season name
// to avoid referencing controls of prior seasons, e.e. CenterstageOperatorControls,
// PowerplayOperatorControls, etc.
public class IntoTheDeepOperatorControls implements PeriodicTask {
    protected RangeInput leftStickX;

    protected RangeInput leftStickY;

    protected RangeInput rightStickX;

    protected RangeInput rightStickY;

    protected OnOffButton dpadUp;

    protected OnOffButton dpadDown;

    protected OnOffButton dpadLeft;

    protected OnOffButton dpadRight;

    protected OnOffButton xBlueButton;

    protected OnOffButton bRedButton;

    protected OnOffButton yYellowButton;

    protected OnOffButton aGreenButton;

    protected OnOffButton rightBumper;

    protected OnOffButton leftBumper;

    protected RangeInput leftTrigger;

    protected RangeInput rightTrigger;

    private NinjaGamePad operatorGamepad;

    private NinjaGamePad driverGamepad;

    private OnOffButton unsafe;

    // FIXME: Add "derived" controls here by name, as they relate to things they control
    // on the robot

    private RangeInput shoulderRotate;

    private RangeInput specimenLiftThrottle;

    private DebouncedButton specimenGripButton;

    private DebouncedButton specimenUngripButton;

    private DebouncedButton specimenStowButton;

    private DebouncedButton specimenHighBarButton;

    private DebouncedButton scoreSpecimenButton;

    private OnOffButton forearmOutButton;

    private OnOffButton forearmInButton;

    private DebouncedButton sampleIntakeButton;

    private DebouncedButton sampleOuttakeButton;

    // FIXME: Add all of the mechanisms controlled by the operator here, and add them to
    // the constructor, and set them there from the constructor arguments

    private SpecimenMechanism specimenMechanism;

    private IntoTheDeepScoringMech scoringMech;

    private boolean noSpecimenAutoUpDown;

    @Builder
    private IntoTheDeepOperatorControls(RangeInput leftStickX,
                                        RangeInput leftStickY,
                                        RangeInput rightStickX,
                                        RangeInput rightStickY,
                                        OnOffButton dpadUp,
                                        OnOffButton dpadDown,
                                        OnOffButton dpadLeft,
                                        OnOffButton dpadRight,
                                        OnOffButton dpadUpRaw,
                                        OnOffButton dpadDownRaw,
                                        OnOffButton dpadLeftRaw,
                                        OnOffButton dpadRightRaw,
                                        OnOffButton xBlueButton,
                                        OnOffButton bRedButton,
                                        OnOffButton yYellowButton,
                                        OnOffButton aGreenButton,
                                        OnOffButton rightBumper,
                                        OnOffButton leftBumper,
                                        RangeInput leftTrigger,
                                        RangeInput rightTrigger,
                                        NinjaGamePad operatorGamepad,
                                        IntoTheDeepScoringMech scoringMech,
                                        SpecimenMechanism specimenMechanism,
                                        boolean noAutoSpecimenUpDown) {
        if (operatorGamepad != null) {
            this.operatorGamepad = operatorGamepad;
            setupFromGamepad();
        } else {
            this.leftStickX = leftStickX;
            this.leftStickY = leftStickY;
            this.rightStickX = rightStickX;
            this.rightStickY = rightStickY;
            this.dpadUp = dpadUp;
            this.dpadDown = dpadDown;
            this.dpadLeft = dpadLeft;
            this.dpadRight = dpadRight;
            this.xBlueButton = xBlueButton;
            this.bRedButton = bRedButton;
            this.yYellowButton = yYellowButton;
            this.aGreenButton = aGreenButton;
            this.rightBumper = rightBumper;
            this.leftBumper = leftBumper;
            this.leftTrigger = leftTrigger;
            this.rightTrigger = rightTrigger;
        }

        this.noSpecimenAutoUpDown = noAutoSpecimenUpDown;

        setupDerivedControls();

        // FIXME: Make sure that mechanisms are setup here, before "wiring" them to
        // controls

        this.scoringMech = scoringMech;
        this.specimenMechanism = specimenMechanism;

        wireControlsToOperatorsMechanisms();
    }


    private void setupFromGamepad() {
        leftStickX = operatorGamepad.getLeftStickX();
        leftStickY = operatorGamepad.getLeftStickY();
        rightStickX = operatorGamepad.getRightStickX();
        rightStickY = operatorGamepad.getRightStickY();

        dpadDown = operatorGamepad.getDpadDown();
        dpadUp = operatorGamepad.getDpadUp();
        dpadLeft = operatorGamepad.getDpadLeft();
        dpadRight = operatorGamepad.getDpadRight();

        aGreenButton = operatorGamepad.getAButton();
        bRedButton = operatorGamepad.getBButton();
        xBlueButton = operatorGamepad.getXButton();
        yYellowButton = operatorGamepad.getYButton();
        leftBumper = operatorGamepad.getLeftBumper();
        rightBumper = operatorGamepad.getRightBumper();
        leftTrigger = operatorGamepad.getLeftTrigger();
        rightTrigger = operatorGamepad.getRightTrigger();
    }

    // FIXME: To make things easier to re-map we have inputs that are named based on what they do
    // and assign them from the gamepad buttons with names that describe physically what they are
    // (bRedButton, etc).
    //
    // It's easier, later on in periodicTask(), to think about the inputs based on functional
    // names.
    private void setupDerivedControls() {
        unsafe = new RangeInputButton( leftTrigger, 0.65f);
        final RangeInput armThrottleCurve = ParametricScaledRangeInput.builder()
                .throttleExponent(13).throttleGain(0.7F).rawInput(leftStickY).build();

        shoulderRotate = armThrottleCurve;

        specimenLiftThrottle = rightStickY;
        specimenGripButton = aGreenButton.debounced();
        specimenUngripButton = bRedButton.debounced();

        specimenHighBarButton = dpadUp.debounced();
        specimenStowButton = dpadDown.debounced();
        scoreSpecimenButton = dpadRight.debounced();

        forearmOutButton = xBlueButton;
        forearmInButton = yYellowButton;

        sampleIntakeButton = new RangeInputButton(rightTrigger, 0.5F).debounced();

        sampleOuttakeButton = rightBumper.debounced();
    }

    // FIXME: As-needed, set controls to setters on scoring mechanisms that have
    //        internal state machines that respond to operator control inputs
    public void wireControlsToOperatorsMechanisms() {
        if (specimenMechanism != null) {
            specimenMechanism.setLiftThrottle(specimenLiftThrottle);
            specimenMechanism.setGripButton(specimenGripButton);
            specimenMechanism.setUngripButton(specimenUngripButton);
            specimenMechanism.setLimitOverrideButton(unsafe);

            if (!noSpecimenAutoUpDown) {
                if (specimenHighBarButton != null) {
                    specimenMechanism.setLiftUpperLimitButton(specimenHighBarButton);
                }

                if (specimenStowButton != null) {
                    specimenMechanism.setLiftLowerLimitButton(specimenStowButton);
                }
            }

            // FIXME
            //specimenMechanism.setLiftLowerLimitButton(autoRetractLiftButton);
            //specimenMechanism.setToFirstLineButton(liftToFirstLineButton);
            // FIXME: Needs an e-stop button!

            if (scoringMech != null && unsafe != null) {
                scoringMech.setUnsafeButton(unsafe);
            }
        }
    }

    @Override
    public void periodicTask() {
        // FIXME: Here is where we ask the various mechanisms to respond to operator input
        float shoulderRotatePosition = shoulderRotate.getPosition();

        if (shoulderRotatePosition < 0) {
            scoringMech.arm.shoulderUp(Math.abs(shoulderRotatePosition));
        } else if (shoulderRotatePosition > 0) {
            scoringMech.arm.shoulderDown(Math.abs(shoulderRotatePosition));
        } else {
            scoringMech.arm.stopShoulder();
        }

        if (forearmInButton.isPressed()) {
            scoringMech.arm.forearmIn();
        } else if (forearmOutButton.isPressed()) {
            scoringMech.arm.forearmOut();
        } else {
            scoringMech.arm.maintainForearmPosition();
        }

        if (sampleIntakeButton.getRise()) {
            scoringMech.arm.intakeSample();
        } else if (sampleOuttakeButton.getRise()) {
            scoringMech.arm.outtakeSample();
        } else {
            scoringMech.arm.maintainIntakeServoPos();
        }

        if (specimenMechanism != null) {
            specimenMechanism.periodicTask();
        }
    }
}
