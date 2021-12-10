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

import com.ftc9929.corelib.control.AnyButton;
import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.RangeInputButton;
import com.ftc9929.corelib.control.ToggledButton;
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

    private FreightManipulator freightManipulator;

    private RangeInput armMotorControl;

    private ToggledButton gripperToggle;

    private CarouselMechanism carouselMechanism;

    private OnOffButton carouselSpinBlue;

    private OnOffButton carouselSpinRed;

    private MaxMotorPowerMagnitude maxMotorPowerMagnitude;

    private DebouncedButton armToLowGoalButton;

    private DebouncedButton armToMidGoalButton;

    private DebouncedButton armToHighGoalButton;

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
                             FreightManipulator freightManipulator,
                             CarouselMechanism carouselMechanism,
                             MaxMotorPowerMagnitude maxMotorPowerMagnitude
                             ) {
        this.freightManipulator = freightManipulator;
        this.carouselMechanism = carouselMechanism;
        this.maxMotorPowerMagnitude = maxMotorPowerMagnitude;

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

    // To make things easier to re-map we have inputs that are named based on what they do
    // and assign them from the gamepad buttons with names that describe physically what they are
    // (bRedButton, etc).
    //
    // It's easier, later on in periodicTask(), to think about the inputs based on functional
    // names.
    private void setupDerivedControls() {
        unsafe = new RangeInputButton( leftTrigger, 0.65f);

        armMotorControl = leftStickY;
        carouselSpinBlue = rightBumper;
        carouselSpinRed = leftBumper;
        gripperToggle = new ToggledButton(operatorGamepad.getAButton());

        armToLowGoalButton = operatorGamepad.getDpadDown().debounced();
        armToHighGoalButton = operatorGamepad.getDpadUp().debounced();

        AnyButton leftRightDpad = new AnyButton(operatorGamepad.getDpadLeft(), operatorGamepad.getDpadRight());

        armToMidGoalButton = leftRightDpad.debounced();

        if (freightManipulator != null) {
            wireControlsToFreightManipulator();
        }
    }

    public void wireControlsToFreightManipulator() {
        freightManipulator.setArmThrottle(armMotorControl);
        freightManipulator.setGripperToggleButton(gripperToggle);
        freightManipulator.setToHubLevelOneButton(armToLowGoalButton);
        freightManipulator.setToHubLevelTwoButton(armToMidGoalButton);
        freightManipulator.setToHubLevelThreeButton(armToHighGoalButton);
        freightManipulator.setUnsafeButton(unsafe);
    }

    @Override
    public void periodicTask() {
        // Here is where we ask the various mechanisms to respond to operator input

        freightManipulator.periodicTask();

        if (maxMotorPowerMagnitude != null) {
            if (maxMotorPowerMagnitude.getMaxPowerMagnitude() > .5) {
                freightManipulator.moveToSafePosition();
            }
        }

        if (carouselSpinRed.isPressed()) {
            carouselMechanism.spinRed();
        } else if (carouselSpinBlue.isPressed()) {
            carouselMechanism.spinBlue();
        } else {
            carouselMechanism.stop();
        }
    }
}
