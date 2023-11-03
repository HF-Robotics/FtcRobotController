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

package com.hfrobots.tnt.season2324;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.RangeInputButton;
import com.hfrobots.tnt.corelib.task.PeriodicTask;

import lombok.Builder;

public class CenterstageOperatorControls implements PeriodicTask {
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

    // Add all of the mechanisms controlled by the operator here, and add them to
    // the constructor.

    protected RangeInput liftThrottle;

    protected DebouncedButton pixelSelector1;

    protected DebouncedButton pixelSelector2;

    protected OnOffButton pixelReleaseButton1;

    protected OnOffButton pixelReleaseButton2;

    protected OnOffButton droneRelease1;

    protected OnOffButton droneRelease2;

    protected RangeInput intakeOutakeThrottle;

    protected OnOffButton hangExtend;

    protected OnOffButton hangRetract;

    protected ScoringMechanism scoringMechanism;

    protected Hanger hanger;

    protected Intake intake;

    @Builder
    private CenterstageOperatorControls(RangeInput leftStickX,
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
                                        NinjaGamePad driverGamepad,
                                        ScoringMechanism scoringMechanism,
                                        Hanger hanger,
                                        Intake intake) {
        if (operatorGamepad != null) {
            this.operatorGamepad = operatorGamepad;
            this.driverGamepad = driverGamepad;

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

        setupDerivedControls();

        this.scoringMechanism = scoringMechanism;
        this.hanger = hanger;
        this.intake = intake;

        wireControlsToScoringMechanism();
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

    // To make things easier to re-map we have inputs that are named based on what they do
    // and assign them from the gamepad buttons with names that describe physically what they are
    // (bRedButton, etc).
    //
    // It's easier, later on in periodicTask(), to think about the inputs based on functional
    // names.
    private void setupDerivedControls() {
        unsafe = new RangeInputButton( leftTrigger, 0.65f);

        liftThrottle = leftStickY;

        pixelSelector1 = leftBumper.debounced();

        pixelSelector2 = rightBumper.debounced();

        pixelReleaseButton1 = xBlueButton;

        pixelReleaseButton2 = bRedButton;

        droneRelease1 = yYellowButton;

        droneRelease2 = new RangeInputButton(rightTrigger, 0.65F);

        intakeOutakeThrottle = rightStickY;

        hangExtend = dpadUp;

        hangRetract = dpadDown;

        wireControlsToScoringMechanism();
    }

    public void wireControlsToScoringMechanism() {
        if (scoringMechanism != null) {
            scoringMechanism.setLiftThrottle(liftThrottle);
            scoringMechanism.setUnsafe(unsafe);
        }
    }

    @Override
    public void periodicTask() {
        // Here is where we ask the various mechanisms to respond to operator input
        if (scoringMechanism != null) {
            scoringMechanism.periodicTask();
        }

        handleHanger();

        handleIntake();
    }

    private void handleIntake() {
        if (intake != null) {
            float throttlePosition = intakeOutakeThrottle.getPosition();

            if (throttlePosition > 0) {
                intake.in(throttlePosition);
            } else if (throttlePosition < 0) {
                intake.out(throttlePosition);
            } else {
                intake.stop();
            }
        }
    }

    private void handleHanger() {
        if (hanger != null) {
            if (hangExtend.isPressed()) {
                hanger.up(1);
            } else if (hangRetract.isPressed()) {
                hanger.down(1);
            } else {
                hanger.stop();
            }
        }
    }
}
