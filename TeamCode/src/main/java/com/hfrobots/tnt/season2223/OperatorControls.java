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

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.RangeInputButton;
import com.hfrobots.tnt.corelib.task.PeriodicTask;

import lombok.Builder;

public class OperatorControls implements PeriodicTask {
    protected RangeInput leftStickX;

    protected RangeInput leftStickY;

    protected RangeInput rightStickX;

    protected RangeInput rightStickY;

    protected DebouncedButton dpadUp;

    protected DebouncedButton dpadDown;

    protected DebouncedButton dpadLeft;

    protected DebouncedButton dpadRight;

    protected OnOffButton xBlueButton;

    protected DebouncedButton bRedButton;

    protected DebouncedButton yYellowButton;

    protected DebouncedButton aGreenButton;

    protected OnOffButton rightBumper;

    protected OnOffButton leftBumper;

    protected RangeInput leftTrigger;

    protected RangeInput rightTrigger;

    private NinjaGamePad operatorGamepad;

    private NinjaGamePad driverGamepad;

    private OnOffButton unsafe;

    // Add all of the mechanisms controlled by the operator here, and add them to
    // the constructor.

    private final LiftMechanism liftMechanism;

    private final Gripper gripper;

    @Builder
    private OperatorControls(RangeInput leftStickX,
                             RangeInput leftStickY,
                             RangeInput rightStickX,
                             RangeInput rightStickY,
                             DebouncedButton dpadUp,
                             DebouncedButton dpadDown,
                             DebouncedButton dpadLeft,
                             DebouncedButton dpadRight,
                             OnOffButton dpadUpRaw,
                             OnOffButton dpadDownRaw,
                             OnOffButton dpadLeftRaw,
                             OnOffButton dpadRightRaw,
                             OnOffButton xBlueButton,
                             DebouncedButton bRedButton,
                             DebouncedButton yYellowButton,
                             DebouncedButton aGreenButton,
                             OnOffButton rightBumper,
                             OnOffButton leftBumper,
                             RangeInput leftTrigger,
                             RangeInput rightTrigger,
                             NinjaGamePad operatorGamepad,
                             NinjaGamePad driverGamepad,
                             LiftMechanism liftMechanism,
                             Gripper gripper) {

        this.liftMechanism = liftMechanism;
        this.gripper = gripper;

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
    }


    private void setupFromGamepad() {
        leftStickX = operatorGamepad.getLeftStickX();
        leftStickY = operatorGamepad.getLeftStickY();
        rightStickX = operatorGamepad.getRightStickX();
        rightStickY = operatorGamepad.getRightStickY();

        dpadDown = new DebouncedButton(operatorGamepad.getDpadDown());
        dpadUp = new DebouncedButton(operatorGamepad.getDpadUp());
        dpadLeft = new DebouncedButton(operatorGamepad.getDpadLeft());
        dpadRight = new DebouncedButton(operatorGamepad.getDpadRight());

        aGreenButton = new DebouncedButton(operatorGamepad.getAButton());
        bRedButton = new DebouncedButton(operatorGamepad.getBButton());
        xBlueButton = operatorGamepad.getXButton();
        yYellowButton = new DebouncedButton(operatorGamepad.getYButton());
        leftBumper = operatorGamepad.getLeftBumper();
        rightBumper = operatorGamepad.getRightBumper();
        leftTrigger = operatorGamepad.getLeftTrigger();
        rightTrigger = operatorGamepad.getRightTrigger();
    }

    RangeInput liftThrottle;

    DebouncedButton liftGoSmallButton;

    DebouncedButton liftGoMediumButton;

    DebouncedButton liftLowerLimitButton;

    DebouncedButton liftUpperLimitButton;

    DebouncedButton liftEmergencyStopButton;

    DebouncedButton gripperOpenButton;

    DebouncedButton gripperCloseButton;

    // To make things easier to re-map we have inputs that are named based on what they do
    // and assign them from the gamepad buttons with names that describe physically what they are
    // (bRedButton, etc).
    //
    // It's easier, later on in periodicTask(), to think about the inputs based on functional
    // names.
    private void setupDerivedControls() {
        unsafe = new RangeInputButton( leftTrigger, 0.65f);
        liftThrottle = leftStickY;
        liftGoSmallButton = dpadLeft;
        liftGoMediumButton = dpadRight;
        liftLowerLimitButton = dpadDown;
        liftUpperLimitButton = dpadUp;
        liftEmergencyStopButton = bRedButton;
        gripperOpenButton = aGreenButton;
        gripperCloseButton = yYellowButton;

        wireControlsToScoringMechanism();
    }

    public void wireControlsToScoringMechanism() {
        if (liftMechanism != null) {
            liftMechanism.setLimitOverrideButton(unsafe);
            liftMechanism.setLiftThrottle(liftThrottle);
            liftMechanism.setLiftGoSmallButton(liftGoSmallButton);
            liftMechanism.setLiftGoMediumButton(liftGoMediumButton);
            liftMechanism.setLiftLowerLimitButton(liftLowerLimitButton);
            liftMechanism.setLiftUpperLimitButton(liftUpperLimitButton);
            liftMechanism.setLiftEmergencyStopButton(liftEmergencyStopButton);
        }
    }

    @Override
    public void periodicTask() {
        // Here is where we ask the various mechanisms to respond to operator input

        if (liftMechanism != null) {
            liftMechanism.doPeriodicTask();
        }

        if (gripperOpenButton.getRise()) {
            gripper.open();
        } else if (gripperCloseButton.getRise()) {
            gripper.close();
        }
    }
}
